"""
Numpy-only port of the PlanT 2.0 input/target pipeline.

Everything in here mirrors code in autonomousvision/plant2 so the records we
produce have exactly the layout the model sees. Each function names the file
and function it was ported from. Nothing here needs torch or CARLA.

Config values match PlanT/config/model/PlanT.yaml (the released config).
"""
import gzip
import json
import os

import numpy as np

# ---------------------------------------------------------------------------
# Config (PlanT/config/model/PlanT.yaml + PlanT/plant_variables.py)
# ---------------------------------------------------------------------------
RANGE = 50                 # training.range  (m, sides/behind)
RANGE_FACTOR_FRONT = 2     # training.range_factor_front (front ellipse = 100 m)
PRECISION_POS = 7          # pre_training.precision_pos   -> 128 bins
PRECISION_ANGLE = 5        # pre_training.precision_angle -> 32 bins
PRECISION_SPEED = 5        # pre_training.precision_speed -> 32 bins
WPS_LEN = 8                # waypoints.wps_len  (4 Hz -> 2 s horizon)
PATH_LEN = 20              # waypoints.path_len (1 m spacing)
SEQ_LEN = 1                # training.seq_len

BEV_COLORS = np.array([
    [0.485, 0.456, 0.406],  # 0 background (ImageNet mean)
    [0.25, 0.25, 0.75],     # 1 road
    [0.485, 0.456, 0.406],  # 2 sidewalk (same as background)
    [0.75, 0.25, 0.25],     # 3 solid lane markings
    [0.25, 0.75, 0.25],     # 4 broken lane markings
], dtype=np.float32)

SPEED_CATS = {50: 0, 80: 1, 100: 2, 120: 3}   # km/h -> speed-limit token index

CLASS_NUMS = {             # object class -> type index (0 = padding)
    "car": 1.0,
    "walker": 2.0,
    "static": 3.0,
    "static_car": 1.0,     # parked cars share the vehicle projection
    "stop_sign": 4.0,
    "traffic_light": 5.0,
    "emergency": 6.0,
}
CAR_TYPES = ["car", "walker", "emergency"]   # "dynamic" classes (carry speed + id)

TARGET_SPEEDS = [0.0, 4.0, 8.0, 10, 13.88888888, 16, 17.77777777, 20]  # m/s, only used by PATH variant

EMERGENCY_TYPE_IDS = [
    "vehicle.dodge.charger_police",
    "vehicle.dodge.charger_police_2020",
    "vehicle.carlamotors.firetruck",
    "vehicle.ford.ambulance",
]
RELEVANT_STATIC_TYPE_IDS = ["static.prop.constructioncone", "static.prop.trafficwarning"]

# Subset of PlanT/util/static_extents.py (CARLA reports wrong extents for statics).
# Pass --plant2-repo to load the full tables instead.
STATIC_EXTENTS = {
    "static.prop.constructioncone": [0.1720348298549652, 0.1720348298549652, 0.2928849756717682],
    "static.prop.trafficwarning": [1.186714768409729, 1.4352929592132568, 1.784761667251587],
}
CAR_EXTENTS = {
    "/Game/Carla/Static/Car/4Wheeled/ParkedVehicles/Lincoln/SM_LincolnParked.SM_LincolnParked":
        [2.44619083404541, 1.115301489830017, 0.7606233954429626],
}


def load_full_extent_tables(repo_root):
    """Replace the embedded extent subset with PlanT/util/static_extents.py."""
    ns = {}
    with open(os.path.join(repo_root, "PlanT", "util", "static_extents.py")) as f:
        exec(f.read(), ns)
    STATIC_EXTENTS.clear(); STATIC_EXTENTS.update(ns["STATIC_EXTENTS"])
    CAR_EXTENTS.clear(); CAR_EXTENTS.update(ns["CAR_EXTENTS"])


# ---------------------------------------------------------------------------
# Small helpers (PlanT/dataset.py)
# ---------------------------------------------------------------------------
def normalize_angle_degree(x):
    x = x % 360.0
    if isinstance(x, np.ndarray):
        x[x > 180] -= 360
    elif x > 180.0:
        x -= 360.0
    return x


def rad2deg(theta):
    return float(normalize_angle_degree(np.rad2deg(theta).item()))


def interpolate_route(points):
    """dataset.interpolate_route: resample route to 20 points 1 m apart (arc length)."""
    points = np.asarray(points, dtype=np.float64)
    route = np.concatenate((np.zeros_like(points[:1]), points))
    shift = np.roll(route, 1, axis=0)
    shift[0] = shift[1]
    dists = np.linalg.norm(route - shift, axis=1)
    dists = np.cumsum(dists)
    dists += np.arange(0, len(dists)) * 1e-4
    x = np.arange(0, 20, 1)
    return np.array([np.interp(x, dists, route[:, 0]), np.interp(x, dists, route[:, 1])]).T


def quantize_box(boxes):
    """dataset.PlanTDataset.quantize_box: [x, y, yaw_deg, speed_kmh] floats -> int bins (-999 kept)."""
    boxes = np.array(boxes, dtype=np.float64)
    if len(boxes) == 0:
        return []
    max_d, max_dd = RANGE, 2 * RANGE
    dummy = boxes == -999
    boxes[:, 0] = (boxes[:, 0] + max_d) / max_dd
    boxes[:, 1] = (boxes[:, 1] + max_d) / max_dd
    boxes[:, 2] = (boxes[:, 2] % 360) / 360
    boxes[:, 3] = boxes[:, 3] / 120
    boxes[:, 0] = np.clip(boxes[:, 0], 0, (1 + RANGE_FACTOR_FRONT) / 2)
    boxes[:, 1:] = np.clip(boxes[:, 1:], 0, 1)
    size_pos, size_speed, size_angle = 2 ** PRECISION_POS, 2 ** PRECISION_SPEED, 2 ** PRECISION_ANGLE
    boxes[:, :2] = (boxes[:, :2] * (size_pos - 1)).round()
    boxes[:, 2] = (boxes[:, 2] * (size_angle - 1)).round()
    boxes[:, 3] = (boxes[:, 3] * (size_speed - 1)).round()
    boxes[dummy] = -999
    return boxes.astype(np.int32).tolist()


def bev_classes_to_tensor(class_map_128):
    """BEV class indices (128x128, values 0..4) -> float32 tensor [3,128,128] (bev_colors lookup)."""
    return BEV_COLORS[class_map_128.astype(np.int64)].transpose(2, 0, 1).astype(np.float32)


def raw_bev_png_to_classes(png_array):
    """dataset.__getitem__ BEV branch: raw 256x256 PNG -> rot90 -> crop [64:-64] -> 128x128 class map."""
    arr = png_array if png_array.ndim == 3 else png_array[None]   # CHW like pil_to_tensor
    arr = np.rot90(arr, axes=(1, 2))
    return np.ascontiguousarray(arr[0, 64:-64, 64:-64])


# ---------------------------------------------------------------------------
# Object encoding (port of the object block in dataset.PlanTDataset.__getitem__)
# ---------------------------------------------------------------------------
def encode_objects(labels_data, input_static_cars=True):
    """
    labels_data: CARLA-style box dicts in the ego frame (ego entry already removed).
    Returns (input_objects_with_id, dropped) where each row is
      [type, x, y, yaw_deg, speed_kmh, width, length, id]
    and dropped is a list of {id, class, reason} for documentation.
    """
    type_nums = dict(CLASS_NUMS)
    if not input_static_cars:
        type_nums.pop("static_car")

    labels_data = [dict(x) for x in labels_data]     # don't mutate caller's data
    dropped = []

    for x in labels_data:
        orig_class = x["class"]
        if "position" in x:
            px, py, pz = x["position"]
            if x["class"] in ["traffic_light", "stop_sign"]:
                if px ** 2 + py ** 2 > 30 ** 2 or abs(pz) > 30:
                    x["class"] = "too far"
            else:
                x_div = RANGE_FACTOR_FRONT ** 2 if px > 0 else 1
                if px ** 2 / x_div + py ** 2 > RANGE ** 2 or abs(pz) > 30:
                    x["class"] = "too far"

        if x["class"] == "car" and x.get("type_id") in EMERGENCY_TYPE_IDS:
            x["class"] = "emergency"
        elif x["class"] == "static":
            if "type_id" in x and x["type_id"] not in RELEVANT_STATIC_TYPE_IDS:
                x["class"] = "irrelevant_static"
            elif x["type_id"] in STATIC_EXTENTS:
                x["extent"] = STATIC_EXTENTS[x["type_id"]]
        elif x["class"] == "static_car":
            if x["mesh_path"] in CAR_EXTENTS:
                x["extent"] = CAR_EXTENTS[x["mesh_path"]]
                if x.get("scale") is not None:
                    x["extent"] = [a * float(x["scale"]) for a in x["extent"]]

        x["_orig_class"] = orig_class

    rows = [
        [
            type_nums[x["class"].lower()],
            x["position"][0],
            x["position"][1],
            rad2deg(x["yaw"]),
            x["speed"] * 3.6,
            x["extent"][1] * 2 + (0 if "scenario" not in x or "Door" not in (x["scenario"] or "") else 1),
            x["extent"][0] * 2,
            x["id"],
        ]
        for x in labels_data
        if x["class"].lower() in CAR_TYPES
    ]

    def keep_static(x):
        c = x["class"].lower()
        if c in CAR_TYPES or c not in type_nums:
            return False
        if c == "traffic_light" and not (x["state"] in ["Red", "Yellow"] and x["affects_ego"]):
            return False
        if c == "stop_sign" and not x["affects_ego"]:
            return False
        return True

    rows += [
        [
            type_nums[x["class"].lower()],
            x["position"][0],
            x["position"][1],
            rad2deg(x["yaw"]),
            0.0,
            x["extent"][1] * 2,
            x["extent"][0] * 2,
            -1 if x["class"].lower() != "static_car" else -999,
        ]
        for x in labels_data
        if keep_static(x)
    ]

    for x in labels_data:
        c = x["class"].lower()
        if c in CAR_TYPES or keep_static(x):
            continue
        if c == "too far":
            reason = "outside input range (50 m circle behind / 100x50 m ellipse ahead; 30 m for lights & stop signs)"
        elif c == "irrelevant_static":
            reason = "static prop that is not a construction cone / traffic warning"
        elif c == "traffic_light":
            reason = ("red/yellow light that does not affect the ego lane" if x.get("state") in ["Red", "Yellow"]
                      else f"{x.get('state')} light (only Red/Yellow lights are input)")
        elif c == "stop_sign":
            reason = "stop sign does not affect ego"
        else:
            reason = f"class '{c}' not in the model's class list"
        dropped.append({"id": x.get("id"), "class": x["_orig_class"], "reason": reason})

    return rows, dropped


def forecast_targets(input_rows, next_frame_objects_in_current_ego):
    """
    dataset.__getitem__ forecasting block. next_frame_objects maps id -> [x, y, yaw_deg, speed_kmh]
    already expressed in the *current* ego frame (that is what output_input_tf does).
    """
    matched = []
    for row in input_rows:
        oid = row[-1]
        if oid == -999:                         # parked car: doesn't move
            matched.append(row[1:5])
        elif oid in next_frame_objects_in_current_ego:
            matched.append(next_frame_objects_in_current_ego[oid])
        else:                                   # lights, stop signs, statics, vanished actors
            matched.append([-999.0, -999.0, -999.0, -999.0])
    return matched, quantize_box(matched)


# ---------------------------------------------------------------------------
# Batching (port of dataset.generate_batch, numpy instead of torch)
# ---------------------------------------------------------------------------
def generate_batch(samples):
    """Returns a dict with the same keys/shapes/dtypes generate_batch builds (as numpy)."""
    maxseq = max(len(s["input"]) for s in samples)
    B = len(samples)
    x_objs = [[0, 0, 0, 0, 0, 0, 0]]          # row 0 = shared padding row
    y_objs = [[-999, -999, -999, -999]]
    idxs = np.zeros((B, maxseq), dtype=np.int32)
    n = 1
    for i, s in enumerate(samples):
        k = len(s["input"])
        idxs[i, :k] = np.arange(n, n + k)
        n += k
        x_objs.extend(s["input"])
        y_objs.extend(s.get("output", [[-999] * 4] * k))
    out = {
        "idxs": idxs,                                            # int32 [B, maxN]
        "x_objs": np.array(x_objs, dtype=np.float32),            # float32 [1+sumN, 7]
        "y_objs": np.array(y_objs, dtype=np.int64),              # int64   [1+sumN, 4]
        "route_original": np.array([s["route_original"] for s in samples], dtype=np.float32),  # [B,20,2]
        "speed_limit": np.array([s["speed_limit"] for s in samples], dtype=np.int32),          # [B]
    }
    if all("BEV" in s for s in samples):
        out["BEV"] = np.stack([s["BEV"] for s in samples]).astype(np.float32)               # [B,3,128,128]
    for key in ["route", "waypoints", "target_speed", "ego_speed"]:
        if all(key in s for s in samples):
            out[key] = np.array([s[key] for s in samples], dtype=np.float32)
    return out


# ---------------------------------------------------------------------------
# Controller-side post-processing (PlanT/PlanT_agent.py::_get_control)
# ---------------------------------------------------------------------------
def desired_speed_from_wps(pred_wps, ego_speed):
    """Target speed the longitudinal controller gets for the path+wps model."""
    pred_wps = np.asarray(pred_wps)
    desired = float(np.linalg.norm(pred_wps[2] - pred_wps[3]) * 4.0)
    mean_speed = float(np.linalg.norm(pred_wps[:-1] - pred_wps[1:], axis=-1).mean() * 4.0)
    creep = ego_speed < 0.01
    if creep:
        desired = min(mean_speed, 0.1)
    return desired, creep


# ---------------------------------------------------------------------------
# Real dataset loader (dataset.PlanTDataset.__getitem__ without augmentation)
# ---------------------------------------------------------------------------
def load_real_sample(route_dir, seq, with_bev=True):
    """
    route_dir: one route folder of the PlanT 2.0 dataset (contains boxes/, measurements/,
    bev_no_car_semantics/). seq: frame index t (needs t+1..t+8 to exist).
    Returns (sample, dropped, raw) in the same structure as the synthetic records.
    """
    from PIL import Image

    def jload(p):
        with gzip.open(p, "rt", encoding="utf-8") as f:
            return json.load(f)

    n = SEQ_LEN + WPS_LEN
    meas = [jload(os.path.join(route_dir, "measurements", f"{seq + i:04d}.json.gz")) for i in range(n)]
    boxes = [jload(os.path.join(route_dir, "boxes", f"{seq + i:04d}.json.gz")) for i in range(n)]
    m0, b0 = meas[SEQ_LEN - 1], boxes[SEQ_LEN - 1]

    mats = [x["ego_matrix"] for x in meas[SEQ_LEN - 1:]]
    ego_inv = np.linalg.inv(mats[0])
    pts = np.array(mats[1:])[:, :, 3]
    waypoints = (ego_inv @ pts.T).T[:, :2].tolist()

    sample = {
        "waypoints": waypoints,
        "route_original": m0["route_original"][:20],
        "route": interpolate_route(m0["route"][:20]).tolist(),
        "target_speed": 0.0 if m0["brake"] else m0["target_speed"],
        "ego_speed": m0["speed"],
        "speed_limit": SPEED_CATS[round(m0["speed_limit"] * 3.6)],
    }
    if with_bev:
        png = np.array(Image.open(os.path.join(route_dir, "bev_no_car_semantics", f"{seq + SEQ_LEN - 1:04d}.png")))
        png = png.transpose(2, 0, 1) if png.ndim == 3 else png[None]
        sample["bev_classes"] = raw_bev_png_to_classes(png)
        sample["BEV"] = bev_classes_to_tensor(sample["bev_classes"])

    rows, dropped = encode_objects(b0[1:])
    # forecasting: next frame boxes transformed into current ego frame
    m1, b1 = meas[SEQ_LEN], boxes[SEQ_LEN]
    tf = np.linalg.inv(np.array(m0["ego_matrix"])) @ np.array(m1["ego_matrix"])
    dyaw = m0["theta"] - m1["theta"]
    nxt = {x["id"]: [*(tf @ np.append(x["position"], [1]))[:2], rad2deg(x["yaw"] - dyaw), x["speed"] * 3.6]
           for x in b1 if "id" in x and "speed" in x}
    floating, quant = forecast_targets(rows, nxt)
    sample["output_floating"] = floating
    sample["output"] = quant
    sample["input_with_ids"] = rows
    sample["input"] = [r[:-1] for r in rows]
    raw = {"measurements": m0, "boxes": b0}
    return sample, dropped, raw
