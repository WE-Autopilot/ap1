"""
Build PlanT 2.0 input/output sample records.

Two modes:
  python make_records.py --out records
      Builds the hand-made scenarios below (synthetic: they did not come from a
      CARLA run, but every number goes through the same pipeline PlanT 2.0 uses).

  python make_records.py --out records_real --route-dir <dataset route folder> \
                         --frames 20 60 100 --plant2-repo <path to plant2 clone>
      Builds records from real PlanT 2.0 dataset frames (HuggingFace SimonGer/PlanT2_Dataset).

Each record = <id>.json + <id>_bev.png (model-ready BEV, RGB uint8; /255 gives the float tensor to within 0.002)
             + <id>_bev.npy (exact float32 [3,128,128], only with --save-npy)
             + <id>_preview.png (what the scene looks like).
"""
import argparse
import json
import math
import os

import numpy as np
from PIL import Image

import plant2_pipeline as P

DT = 0.25          # 4 Hz between waypoints (20 fps / data_save_freq 5)
PX = 0.5           # BEV metres per pixel
N = 128            # BEV size in pixels
LANE = 3.5         # lane width used for the hand-made road layouts

LINCOLN = "/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Lincoln/SM_LincolnParked.SM_LincolnParked"
EGO_EXTENT = [2.44619083404541, 1.0005781650543213, 0.7606233954429626]
CAR_EXTENT = [2.3958897590637207, 1.081725001335144, 0.7438300848007202]
WALKER_EXTENT = [0.18767888844013214, 0.18767888844013214, 0.9300000071525574]
AMBULANCE_EXTENT = [3.18, 1.1923, 1.235]


# ---------------------------------------------------------------------------
# BEV drawing for the hand-made scenes (class map: 0 bg, 1 road, 2 sidewalk, 3 solid, 4 broken)
# Orientation: row 0 = 32 m ahead of ego, col 0 = 32 m to the left (ego faces up),
# which is how PlanT/util/viz_batch.py pastes the BEV under the boxes.
# ---------------------------------------------------------------------------
def grid():
    r = np.arange(N)
    xs = (N / 2 - 0.5 - r) * PX        # forward metres per row
    ys = (r - N / 2 + 0.5) * PX        # right metres per column
    return np.meshgrid(xs, ys, indexing="ij")


def draw_road_along_x(cm, y_left, y_right, centre_y, x_from=-99, x_to=99, dashed=True):
    """Road running forward/back between y_left..y_right with a solid centre divider at centre_y."""
    X, Y = grid()
    band = (X >= x_from) & (X <= x_to)
    cm[band & (Y >= y_left - 3) & (Y <= y_right + 3) & (cm == 0)] = 2            # sidewalks
    cm[band & (Y >= y_left) & (Y <= y_right)] = 1                               # road
    for edge in (y_left, y_right, centre_y):
        cm[band & (np.abs(Y - edge) < PX * 0.6)] = 3                            # solid lines
    if dashed:
        dash = (np.floor(X / 3.0) % 2) == 0
        for y in np.arange(y_left + LANE, y_right - 0.1, LANE):
            if abs(y - centre_y) > 0.1:
                cm[band & dash & (np.abs(Y - y) < PX * 0.6)] = 4                # broken lines


def draw_road_along_y(cm, x_near, x_far, centre_x):
    X, Y = grid()
    cm[(X >= x_near - 3) & (X <= x_far + 3) & (cm == 0)] = 2
    cm[(X >= x_near) & (X <= x_far)] = 1
    for edge in (x_near, x_far, centre_x):
        cm[np.abs(X - edge) < PX * 0.6] = 3        # junction interior is cleared afterwards


def clear_junction(cm, x_near, x_far, y_left, y_right):
    X, Y = grid()
    cm[(X >= x_near) & (X <= x_far) & (Y >= y_left) & (Y <= y_right)] = 1


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def polyline_at(poly, s):
    poly = np.asarray(poly, dtype=float)
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seg)])
    s = np.clip(s, 0, cum[-1])
    return np.array([np.interp(s, cum, poly[:, 0]), np.interp(s, cum, poly[:, 1])]).T


def lane_change_path(y_end, start=8.0, length=18.0, total=80.0):
    xs = np.arange(0, total, 0.5)
    t = np.clip((xs - start) / length, 0, 1)
    ys = y_end * (3 * t ** 2 - 2 * t ** 3)
    return np.stack([xs, ys], 1)


def straight_path(total=80.0):
    return np.stack([np.arange(0, total, 0.5), np.zeros(int(total / 0.5))], 1)


def route_from_path(path):
    """autopilot.py: route points start 2.5 m ahead (tf_first_checkpoint_distance) and are 1 m apart."""
    return polyline_at(path, 2.5 + np.arange(20)).tolist()


def ego_waypoints(path, v0, accel, v_min=0.0, stop_at=None):
    """Expert future positions at t = 0.25..2.0 s along `path` with constant accel (clamped)."""
    s, v, out = 0.0, v0, []
    for _ in range(P.WPS_LEN):
        for _ in range(25):                         # integrate in 10 ms steps
            v = max(v_min, v + accel * 0.01)
            s += v * 0.01
        if stop_at is not None:
            s = min(s, stop_at)
        out.append(polyline_at(path, s).tolist())
    return out


def car(i, x, y, yaw_deg, speed, type_id="vehicle.tesla.model3", extent=CAR_EXTENT, scenario=""):
    return {"class": "car", "id": i, "type_id": type_id, "position": [x, y, 0.0],
            "yaw": math.radians(yaw_deg), "speed": speed, "extent": list(extent),
            "num_points": -1, "role_name": "scenario" if scenario else "background", "scenario": scenario}


def walker(i, x, y, yaw_deg, speed):
    return {"class": "walker", "id": i, "position": [x, y, 0.0], "yaw": math.radians(yaw_deg),
            "speed": speed, "extent": list(WALKER_EXTENT), "num_points": -1}


def static(i, x, y, yaw_deg, type_id, extent=(0.5, 0.5, 0.5)):
    return {"class": "static", "id": i, "type_id": type_id, "position": [x, y, 0.0],
            "yaw": math.radians(yaw_deg), "extent": list(extent)}


def static_car(i, x, y, yaw_deg, mesh=LINCOLN):
    return {"class": "static_car", "id": i, "mesh_path": mesh, "position": [x, y, 0.0],
            "yaw": math.radians(yaw_deg), "extent": [2.0, 1.0, 0.7], "scale": None}


def light(x, y, state, affects_ego, yaw_deg=0.0):
    return {"class": "traffic_light", "position": [x, y, 0.0], "yaw": math.radians(yaw_deg),
            "extent": [1.5, 1.5, 0.5], "state": state, "affects_ego": affects_ego}


def stop_sign(x, y, affects_ego, yaw_deg=0.0):
    return {"class": "stop_sign", "position": [x, y, 0.0], "yaw": math.radians(yaw_deg),
            "extent": [1.5, 1.5, 0.5], "affects_ego": affects_ego}


def ego_box(speed):
    return {"class": "ego_car", "id": 0, "position": [0.0, 0.0, 0.0], "yaw": 0.0, "speed": speed,
            "extent": EGO_EXTENT, "num_points": -1, "distance": -1, "brake": 0.0}


# ---------------------------------------------------------------------------
# Hand-made scenarios
# ---------------------------------------------------------------------------
def scenarios():
    S = []

    # 1. Urban two-lane road, following a slower lead car, oncoming traffic -----------------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 0.5 * LANE, -0.5 * LANE)
    path = straight_path()
    S.append(dict(
        id="01_urban_follow_lead",
        title="Urban road, following a slower lead car (50 km/h zone)",
        speed_limit=50 / 3.6, ego_speed=11.0, target_speed=9.0, brake=False,
        boxes=[car(101, 18.0, 0.0, 0.0, 9.0),
               car(102, 35.0, -LANE, 180.0, 12.0),
               car(103, -12.0, 0.0, 0.0, 11.5)],
        route=route_from_path(path), route_original=route_from_path(path),
        wps=ego_waypoints(path, 11.0, -1.2), bev=cm))

    # 2. Red light at a signalised junction ------------------------------------------------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 0.5 * LANE, -0.5 * LANE)
    draw_road_along_y(cm, 16.0, 23.0, 19.5)
    clear_junction(cm, 16.0, 23.0, -1.5 * LANE, 0.5 * LANE)
    path = straight_path()
    stop_line = 13.5
    S.append(dict(
        id="02_red_light_stop",
        title="Approaching a red light: stop-line box makes the expert brake to a stop",
        speed_limit=50 / 3.6, ego_speed=8.0, target_speed=0.0, brake=True,
        boxes=[light(stop_line, 0.0, "Red", True),
               light(stop_line, -LANE, "Red", False),        # opposite-direction light -> dropped
               light(26.0, LANE, "Green", False),             # green -> dropped
               car(201, 19.5, -12.0, 90.0, 10.0),             # cross traffic with green
               car(202, 9.0, -LANE, 180.0, 0.0)],             # oncoming car waiting
        route=route_from_path(path), route_original=route_from_path(path),
        wps=ego_waypoints(path, 8.0, -8.0 ** 2 / (2 * (stop_line - 3.0)), stop_at=stop_line - 3.0), bev=cm))

    # 3. Pedestrian crossing in front of the ego ------------------------------------------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 0.5 * LANE, -0.5 * LANE)
    path = straight_path()
    S.append(dict(
        id="03_pedestrian_crossing",
        title="Pedestrian crossing the ego lane; a standing pedestrian on the sidewalk",
        speed_limit=50 / 3.6, ego_speed=7.0, target_speed=0.0, brake=True,
        boxes=[walker(301, 12.0, 4.5, -90.0, 1.4),            # walking right->left into the lane
               walker(302, 6.0, -5.5, 0.0, 0.0),              # standing (dataset keeps it; agent drops it)
               car(303, 30.0, -LANE, 180.0, 10.0)],
        route=route_from_path(path), route_original=route_from_path(path),
        wps=ego_waypoints(path, 7.0, -7.0 ** 2 / (2 * 7.5), stop_at=7.5), bev=cm))

    # 4. Construction zone blocking the lane: route is shifted into the oncoming lane ------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 0.5 * LANE, -0.5 * LANE)
    detour = lane_change_path(-LANE, start=6.0, length=14.0)
    path_orig = straight_path()
    cones = [static(400 + k, 18.0 + 1.5 * k, 0.3 * (k % 3) - 0.5, 0.0, "static.prop.constructioncone")
             for k in range(8)]
    S.append(dict(
        id="04_construction_detour",
        title="Construction zone in ego lane: expert detours into the oncoming lane",
        speed_limit=50 / 3.6, ego_speed=6.0, target_speed=6.0, brake=False,
        boxes=cones + [static(420, 16.0, 0.0, 0.0, "static.prop.trafficwarning"),
                       static(421, 10.0, 6.5, 0.0, "static.prop.barbeque"),   # irrelevant prop -> dropped
                       car(422, 70.0, -LANE, 180.0, 8.0)],
        route=route_from_path(detour), route_original=route_from_path(path_orig),
        wps=ego_waypoints(detour, 6.0, 0.0), bev=cm))

    # 5. Ambulance approaching from behind on a highway -------------------------------------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 1.5 * LANE, -1.5 * LANE)                 # 3 same-direction lanes
    side = lane_change_path(LANE, start=4.0, length=20.0)
    S.append(dict(
        id="05_emergency_vehicle_yield",
        title="Ambulance approaching from behind at 100 km/h: expert moves one lane right",
        speed_limit=100 / 3.6, ego_speed=24.0, target_speed=22.0, brake=False,
        boxes=[car(501, -28.0, 0.0, 0.0, 31.0, type_id="vehicle.ford.ambulance", extent=AMBULANCE_EXTENT,
                   scenario="YieldToEmergencyVehicle_1"),
               car(502, 40.0, -LANE, 0.0, 26.0),
               car(503, 60.0, LANE, 0.0, 21.0)],
        route=route_from_path(side), route_original=route_from_path(straight_path()),
        wps=ego_waypoints(side, 24.0, -0.8), bev=cm))

    # 6. Stop sign at a junction, parked car at the kerb, a far-away car ------------------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 0.5 * LANE, -0.5 * LANE)
    draw_road_along_y(cm, 14.0, 21.0, 17.5)
    clear_junction(cm, 14.0, 21.0, -1.5 * LANE, 0.5 * LANE)
    path = straight_path()
    S.append(dict(
        id="06_stop_sign_parked_car",
        title="Stop sign ahead, parked car at the kerb, car beyond the input range",
        speed_limit=50 / 3.6, ego_speed=5.0, target_speed=0.0, brake=True,
        boxes=[stop_sign(11.5, 0.0, True),
               stop_sign(23.5, -LANE, False),                       # other approach's sign -> dropped
               static_car(601, 6.0, 0.5 * LANE + 1.2, 0.0),
               car(602, 17.5, 14.0, -90.0, 7.0),
               car(603, 120.0, 0.0, 0.0, 10.0)],                    # x=120 > 100 m ellipse -> dropped
        route=route_from_path(path), route_original=route_from_path(path),
        wps=ego_waypoints(path, 5.0, -5.0 ** 2 / (2 * 8.5), stop_at=8.5), bev=cm))

    # 7. Highway cut-in at 120 km/h ---------------------------------------------------------
    cm = np.zeros((N, N), np.uint8)
    draw_road_along_x(cm, -1.5 * LANE, 1.5 * LANE, -1.5 * LANE)                   # 3 same-direction lanes
    path = straight_path(120)
    S.append(dict(
        id="07_highway_cut_in",
        title="Car cutting in from the right lane at 120 km/h; expert brakes moderately",
        speed_limit=120 / 3.6, ego_speed=30.0, target_speed=25.0, brake=False,
        boxes=[car(701, 14.0, 2.4, -8.0, 26.0, scenario="HighwayCutIn_1"),
               car(702, 85.0, 0.0, 0.0, 29.0),                       # 85 m ahead: inside the ellipse
               car(703, -20.0, LANE, 0.0, 31.0)],
        route=route_from_path(path), route_original=route_from_path(path),
        wps=ego_waypoints(path, 30.0, -3.0), bev=cm))

    return S


# ---------------------------------------------------------------------------
# Record assembly
# ---------------------------------------------------------------------------
def next_frame(boxes):
    """Move dynamic actors forward 0.25 s along their heading (constant velocity), in the current ego frame."""
    out = {}
    for b in boxes:
        if "speed" not in b or "id" not in b:
            continue
        x, y = b["position"][:2]
        out[b["id"]] = [x + b["speed"] * DT * math.cos(b["yaw"]), y + b["speed"] * DT * math.sin(b["yaw"]),
                        P.rad2deg(b["yaw"]), b["speed"] * 3.6]
    return out


def rnd(x, d=4):
    if isinstance(x, (list, tuple)):
        return [rnd(v, d) for v in x]
    if isinstance(x, (float, np.floating)):
        return round(float(x), d)
    if isinstance(x, np.integer):
        return int(x)
    return x


def build_synthetic(sc):
    boxes = [ego_box(sc["ego_speed"])] + sc["boxes"]
    rows, dropped = P.encode_objects(boxes[1:])
    floating, quant = P.forecast_targets(rows, next_frame(sc["boxes"]))
    sample = {
        "input": [r[:-1] for r in rows],
        "input_with_ids": rows,
        "output": quant,
        "output_floating": floating,
        "route_original": sc["route_original"],
        "route": P.interpolate_route(sc["route"]).tolist(),
        "waypoints": sc["wps"],
        "speed_limit": P.SPEED_CATS[round(sc["speed_limit"] * 3.6)],
        "target_speed": 0.0 if sc["brake"] else sc["target_speed"],
        "ego_speed": sc["ego_speed"],
        "bev_classes": sc["bev"],
        "BEV": P.bev_classes_to_tensor(sc["bev"]),
    }
    raw = {
        "measurements": {
            "speed": sc["ego_speed"], "speed_limit": sc["speed_limit"], "target_speed": sc["target_speed"],
            "brake": sc["brake"], "theta": 0.0,
            "route": sc["route"], "route_original": sc["route_original"],
            "changed_route": sc["route"] != sc["route_original"],
            "augmentation_translation": 0.0, "augmentation_rotation": 0.0,
        },
        "boxes": boxes,
    }
    return sample, dropped, raw


def write_record(out_dir, rid, title, sample, dropped, raw, provenance, save_npy=False):
    os.makedirs(out_dir, exist_ok=True)
    bev_png = f"{rid}_bev.png"
    bev_npy = f"{rid}_bev.npy" if save_npy else None
    Image.fromarray((sample["BEV"].transpose(1, 2, 0) * 255).round().astype(np.uint8)).save(os.path.join(out_dir, bev_png))
    if save_npy:
        np.save(os.path.join(out_dir, bev_npy), sample["BEV"])

    desired, creep = P.desired_speed_from_wps(sample["waypoints"], sample["ego_speed"])
    ids = [r[-1] for r in sample["input_with_ids"]]
    record = {
        "record_id": rid,
        "title": title,
        "provenance": provenance,
        "raw": rnd(raw),
        "model_input": {
            "x_objs": {
                "columns": ["type", "x_m", "y_m", "yaw_deg", "speed_kmh", "width_m", "length_m"],
                "dtype": "float32", "shape": [len(sample["input"]), 7],
                "rows": rnd(sample["input"]),
                "source_actor_ids": ids,
            },
            "route_original": {"dtype": "float32", "shape": [20, 2], "points_xy_m": rnd(sample["route_original"])},
            "speed_limit": {"dtype": "int", "value": sample["speed_limit"],
                            "kmh": {v: k for k, v in P.SPEED_CATS.items()}[sample["speed_limit"]]},
            "BEV": {"dtype": "float32", "shape": [3, 128, 128], "png": bev_png, "npy": bev_npy,
                    "class_pixel_counts": {name: int((sample["bev_classes"] == i).sum()) for i, name in
                                           enumerate(["background", "road", "sidewalk", "solid_line", "broken_line"])}},
            "dropped_objects": dropped,
        },
        "model_output": {
            "note": "Training targets (expert labels): what PlanT 2.0 learns to output for this input.",
            "pred_path": {"dtype": "float32", "shape": [20, 2], "points_xy_m": rnd(sample["route"])},
            "pred_wps": {"dtype": "float32", "shape": [8, 2], "dt_s": DT, "points_xy_m": rnd(sample["waypoints"])},
            "pred_speed": None,
            "forecast_y_objs": {
                "note": "Auxiliary training target only (not used for driving): each input object's state "
                        "0.25 s later, quantised. -999 = no target (lights, stop signs, statics).",
                "columns": ["x_bin(0-191)", "y_bin(0-127)", "yaw_bin(0-31)", "speed_bin(0-31)"],
                "dtype": "int64", "shape": [len(sample["output"]), 4],
                "rows": sample["output"],
                "rows_unquantised": rnd(sample["output_floating"], 2),
            },
        },
        "controller": {
            "desired_speed_mps": round(desired, 3),
            "creep_mode": creep,
            "expert_target_speed_mps": rnd(sample["target_speed"]),
            "ego_speed_mps": rnd(sample["ego_speed"]),
        },
    }
    with open(os.path.join(out_dir, f"{rid}.json"), "w") as f:
        json.dump(record, f, indent=1)
    render_preview(os.path.join(out_dir, f"{rid}_preview.png"), title, sample)
    return record


def render_preview(path, title, sample):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    colours = {1: "#d62728", 2: "#17becf", 3: "#7f7f7f", 4: "#e377c2", 5: "#ff7f0e", 6: "#9467bd"}
    names = {1: "vehicle / parked", 2: "pedestrian", 3: "static", 4: "stop sign", 5: "traffic light", 6: "emergency"}
    fig, ax = plt.subplots(figsize=(5.2, 7))
    ax.imshow(sample["BEV"].transpose(1, 2, 0), extent=[-32, 32, -32, 32], origin="upper", zorder=0)
    ax.add_patch(Polygon([[-1.0, -2.45], [1.0, -2.45], [1.0, 2.45], [-1.0, 2.45]], closed=True,
                         fc="white", ec="black", zorder=3, label="ego"))
    seen = set()
    for r in sample["input"]:
        t, x, y, yaw, v, w, l = r
        a = math.radians(yaw)
        f, s = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        c = [np.array([x, y]) + sx * f * l / 2 + sy * s * w / 2 for sx, sy in [(1, 1), (1, -1), (-1, -1), (-1, 1)]]
        ax.add_patch(Polygon([[p[1], p[0]] for p in c], closed=True, fc=colours[int(t)], ec="black", alpha=0.85,
                             zorder=4, label=None if t in seen else names[int(t)]))
        seen.add(t)
        if v > 0:
            d = f * v / 3.6 * 0.5
            ax.arrow(y, x, d[1], d[0], width=0.15, color="black", zorder=5)
    ro, pa, wp = map(np.array, (sample["route_original"], sample["route"], sample["waypoints"]))
    ax.plot(ro[:, 1], ro[:, 0], "o", ms=3, color="#1f77b4", label="input: route_original", zorder=6)
    ax.plot(pa[:, 1], pa[:, 0], "-", lw=2, color="#ff4500", label="output: path (20 pts)", zorder=7)
    ax.plot(wp[:, 1], wp[:, 0], "s", ms=4, color="#2ca02c", mec="black", label="output: waypoints (8 @ 4 Hz)", zorder=8)
    xmax = max(40, float(np.max([r[1] for r in sample["input"]] + [30])) + 8)
    ax.set_xlim(-30, 30); ax.set_ylim(-35, min(xmax, 105))
    ax.set_xlabel("y (m, right +)"); ax.set_ylabel("x (m, forward +)")
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=6.5, loc="lower left", framealpha=0.9)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--route-dir", help="real dataset route folder (boxes/, measurements/, bev_no_car_semantics/)")
    ap.add_argument("--frames", type=int, nargs="*", default=[20, 60, 100])
    ap.add_argument("--plant2-repo", help="plant2 clone, to load the full static extent tables")
    ap.add_argument("--save-npy", action="store_true", help="also write the exact float32 BEV tensor as .npy")
    args = ap.parse_args()
    if args.plant2_repo:
        P.load_full_extent_tables(args.plant2_repo)

    built = []
    if args.route_dir:
        name = os.path.basename(args.route_dir.rstrip("/"))
        for t in args.frames:
            sample, dropped, raw = P.load_real_sample(args.route_dir, t)
            rid = f"{name}_{t:04d}"
            built.append((rid, write_record(args.out, rid, f"{name} frame {t}", sample, dropped, raw,
                                            f"PlanT 2.0 dataset, {args.route_dir}, frame {t}", args.save_npy), sample))
    else:
        for sc in scenarios():
            sample, dropped, raw = build_synthetic(sc)
            built.append((sc["id"], write_record(
                args.out, sc["id"], sc["title"], sample, dropped, raw,
                "Hand-built scene (not from a CARLA run), processed with the PlanT 2.0 input pipeline "
                "(ported from PlanT/dataset.py). Outputs are expert-style targets, not model predictions.", args.save_npy), sample))

    batch = P.generate_batch([s for _, _, s in built])
    summary = {
        "records": [rid for rid, _, _ in built],
        "batch_shapes": {k: {"shape": list(v.shape), "dtype": str(v.dtype)} for k, v in batch.items()},
        "batch_idxs": batch["idxs"].tolist(),
        "x_objs_first_rows": rnd(batch["x_objs"][:6].tolist()),
    }
    with open(os.path.join(args.out, "batch_example.json"), "w") as f:
        json.dump(summary, f, indent=1)
    for rid, rec, _ in built:
        print(f"{rid}: {rec['model_input']['x_objs']['shape'][0]} objects, "
              f"{len(rec['model_input']['dropped_objects'])} dropped, speed_limit idx {rec['model_input']['speed_limit']['value']}, "
              f"desired {rec['controller']['desired_speed_mps']} m/s")
    print("batch:", summary["batch_shapes"])


if __name__ == "__main__":
    main()
