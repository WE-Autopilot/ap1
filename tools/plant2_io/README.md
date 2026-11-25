# PlanT 2.0 — inputs & outputs reference

What goes into PlanT 2.0, what comes out, and how the outputs turn into vehicle control. Each item is traced to the code in [autonomousvision/plant2](https://github.com/autonomousvision/plant2) (commit `6269e5a`) and to the PlanT 2.0 paper (arXiv 2511.07292). Shapes and values come from the released config, `PlanT/config/model/PlanT.yaml`.

```
CARLA world ──► boxes + measurements ──► PlanTDataset / PlanTAgent.get_input_batch ──► generate_batch ──► HFLM.forward ──► (path, waypoints) ──► controllers ──► carla.VehicleControl
                (privileged ground truth)    (filter + encode objects, BEV, route)       (pad/batch)         (BERT-medium)
```

**Contents**
1. Conventions
2. Inputs: object matrix, BEV image, route, speed limit
3. How the inputs become a token sequence
4. Outputs: path, waypoints, forecasting head, attention
5. From outputs to vehicle control
6. Code samples
7. Sample records (`records/`)
8. Gotchas

---

## 1. Conventions

| | |
|---|---|
| Frame | **Ego-relative**, origin at the ego vehicle's centre, at the current frame |
| Axes | **x = forward**, **y = right** (CARLA's left-handed frame), z = up (z is used only for filtering) |
| Yaw | relative to ego heading. **Radians** in raw boxes, **degrees** in model input, normalised to (-180, 180]. Positive = turned right |
| Speed | **m/s** in raw data. Object speeds are converted to **km/h** in the model input |
| Rate | Data is saved at **4 Hz** (CARLA 20 fps, `data_save_freq = 5`), so waypoints are 0.25 s apart |
| Perception | None. PlanT gets **ground-truth** boxes from the simulator (privileged planner) |

---

## 2. Inputs

The model's `forward(batch)` reads these keys (`PlanT/model.py`):

| Key | Type / shape (batched) | Meaning |
|---|---|---|
| `x_objs` | `float32 [1 + ΣN, 7]` | All objects of all samples stacked, plus a padding row at index 0 |
| `idxs` | `int32 [B, maxN]` | For each sample, the rows of `x_objs` that belong to it (0 = padding) |
| `BEV` | `float32 [B, 3, 128, 128]` | Road-layout image (SD map). Used because `input_bev: True` |
| `route_original` | `float32 [B, 20, 2]` | 20 route points, 1 m apart |
| `speed_limit` | `int32 [B]` | Speed-limit category 0–3 |
| `input_ego_speed` | `float32 [B]` | **Off** in the released config (`input_ego_speed: False`) |
| `y_objs` | `int64 [1 + ΣN, 4]` or `None` | Forecasting targets. Training only; `None` at inference |

### 2.1 Object matrix — "other agents, lights & signs"

Each object becomes **one row of 7 floats**:

| col | name | unit | notes |
|---|---|---|---|
| 0 | `type` | class index | see table below. The model strips this column and uses it to pick a **class-specific linear projection** |
| 1 | `x` | m | forward |
| 2 | `y` | m | right |
| 3 | `yaw` | deg | relative to ego |
| 4 | `speed` | km/h | always 0 for static classes |
| 5 | `width` | m | `extent[1] * 2`. **+1 m** for a vehicle whose scenario name contains `"Door"` (door-opening scenario) |
| 6 | `length` | m | `extent[0] * 2` |

> The comment in `model.py` says `x, y, yaw, speed/id, extent x, extent y`, but the code puts the **y-extent (width) first**: col 5 = width, col 6 = length.

**Classes** (`PlanT/plant_variables.py`):

| index | class | taken from | included when… |
|---|---|---|---|
| 0 | padding | — | — |
| 1 | `car` | every `*vehicle*` actor | inside the range (below) |
| 1 | `static_car` (parked) | parked-car meshes | `input_static_cars: True`. Extents come from the `CAR_EXTENTS` table because CARLA reports wrong ones |
| 2 | `walker` | `*walker*` actors | in range. **At inference** a pedestrian only appears once it has moved (speed ≥ 0.1 m/s at least once) |
| 3 | `static` | static props | only `static.prop.constructioncone` and `static.prop.trafficwarning`. Extents come from `STATIC_EXTENTS` |
| 4 | `stop_sign` | a box on the stop line | affects the ego, within 30 m, not yet cleared |
| 5 | `traffic_light` | a box on the stop line (one per lane waypoint) | state **Red or Yellow** and affects the ego, within 30 m |
| 6 | `emergency` | vehicles with `type_id` in {`dodge.charger_police`, `dodge.charger_police_2020`, `carlamotors.firetruck`, `ford.ambulance`} | in range |

Lights and stop signs are fixed 3 × 3 m boxes (`extent [1.5, 1.5, 0.5]`). They mark **where the car must stop**, not where the pole stands.

**Input range** (`range: 50`, `range_factor_front: 2`):
- behind the ego (x ≤ 0): circle of **50 m**
- ahead (x > 0): ellipse, **100 m** along x and **50 m** along y: `x²/4 + y² ≤ 50²`
- lights and stop signs: circle of **30 m**
- anything with |z| > 30 m is dropped

Row order: dynamic objects (car, walker, emergency) first, then lights, signs, statics and parked cars. The number of objects varies per sample, which is why batches use the flat `x_objs` + `idxs` layout.

### 2.2 Bird's-eye-view image (SD map)

| | |
|---|---|
| Shape / dtype | `float32 [3, 128, 128]` (CHW) |
| Coverage | **64 m × 64 m**, centred on the ego, **0.5 m per pixel** |
| Source | `ObsManager` (`carla_garage/birds_eye_view/chauffeurnet.py`) renders a 256 × 256 class map at 2 px/m. The training data stores it as `bev_no_car_semantics/XXXX.png` |
| Processing | `rot90` → crop `[64:-64, 64:-64]` → look up each class index in `bev_colors` |
| Encoder | ResNet-18 (ImageNet-pretrained, `num_classes=512`) → **1 token** |
| Orientation | Ego faces **up** (row 0 is 32 m ahead) and +y points right. This is how `util/viz_batch.py` lays the image under the boxes |

Class → RGB (each channel 0–1):

| idx | class | colour |
|---|---|---|
| 0 | background | ImageNet mean (0.485, 0.456, 0.406) |
| 1 | drivable road | (0.25, 0.25, 0.75) blue |
| 2 | sidewalk | same as background (the model can't tell them apart) |
| 3 | solid lane marking | (0.75, 0.25, 0.25) red |
| 4 | broken lane marking | (0.25, 0.75, 0.25) green |

The image has **no cars**. It tells the model which lanes go which way and how many lanes there are.

### 2.3 Route

| | |
|---|---|
| Key | `route_original` |
| Shape | `float32 [20, 2]`, (x, y) in m |
| Sampling | starts **2.5 m ahead** of the ego (`tf_first_checkpoint_distance`), then one point **every 1 m** |
| Encoder | flattened to 40 numbers → `Linear(40, 512)` → **1 token** |

There are two routes in the data:
- **`route_original`** is the GPS route from the leaderboard. This is the model **input**.
- **`route`** is the expert's adjusted route, e.g. a detour around a construction zone. It is used as the **path target**.

### 2.4 Speed limit

| | |
|---|---|
| Value | `int` in {0, 1, 2, 3} ↔ {50, 80, 100, 120} km/h. Computed as `speed_cats[round(speed_limit_mps * 3.6)]` |
| Encoder | `nn.Embedding(4, 512)` → **1 token** |

---

## 3. How the inputs become a token sequence

From `HFLM.forward`, with the released config (`path+wps`, `linear` generator):

```
[ 8 waypoint query tokens | 20 path query tokens | BEV | speed limit | route | obj_1 … obj_N ]
  └──────── learned (28) ────────────────────────┘  └── 1 ┘ └── 1 ──┘  └ 1 ┘  └─ class-specific Linear(6→512)
```

The sequence goes through a **BERT-medium** encoder (`prajjwal1/bert-medium`: 8 layers, hidden size 512, 8 heads), built from its config, i.e. not pretrained. Its built-in word embeddings and pooler are removed. The first 28 output tokens become the plan. The object tokens (from index 31 on) feed the forecasting heads.

---

## 4. Outputs

`HFLM.forward` returns `(logits, targets, (pred_path, pred_wps, pred_speed), attn_map)`.

| Output | Shape | Unit | How it is made | Used for |
|---|---|---|---|---|
| `pred_wps` | `float32 [B, 8, 2]` | m, ego frame | Linear(512→2) on each of the 8 waypoint tokens, then **cumulative sum** | **Longitudinal** control (speed). Points are 0.25 s apart, so they cover 2 s |
| `pred_path` | `float32 [B, 20, 2]` | m, ego frame | Linear(512→2) on each of the 20 path tokens, then cumsum | **Lateral** control (steering). Spatial points meant to be ~1 m apart; the target starts at (0, 0) |
| `pred_speed` | `None` | — | Only exists in the `path+2hot` variant (8-way classifier over `target_speeds`) | not used in the release |
| `logits` | list of 4 × `[ΣN·…, vocab]` | bins | 4 linear heads on the object tokens. Vocab sizes **[192, 128, 32, 32]** for x, y, yaw, speed | **Training only.** Auxiliary task: predict each object's state 0.25 s ahead. `None` when `y_objs` is `None` |
| `attn_map` | tuple of 8 × `[B, 8, T, T]` | — | BERT attentions | explainability / visualisation |

**Training targets** (`lit_module.py`): `waypoints` = the expert's next 8 positions (from `ego_matrix` at t+1…t+8). `route` = the expert's adjusted route resampled to 20 points 1 m apart (`interpolate_route`). The losses are L1 on both, weighted path × 5 and waypoints × 1, plus cross-entropy on the forecasting bins.

**How forecasting targets are quantised** (`quantize_box`):

| attribute | normalise | bins |
|---|---|---|
| x | `(x + 50) / 100`, clipped to [0, 1.5] | 128 × 1.5 → **0–191** (≈ 0.79 m per bin) |
| y | `(y + 50) / 100`, clipped to [0, 1] | **0–127** |
| yaw | `(yaw mod 360) / 360` | **0–31** (11.25° per bin) |
| speed | `speed_kmh / 120` | **0–31** (≈ 3.9 km/h per bin) |

`-999` marks objects with no target (lights, stop signs, statics, actors that disappeared). Parked cars use their current pose as the target.

---

## 5. From outputs to vehicle control

From `PlanT_agent.py::_get_control`:

1. **Desired speed** = `‖wps[2] − wps[3]‖ × 4` m/s, i.e. the distance between the 3rd and 4th waypoints divided by 0.25 s.
   *Creep heuristic:* if the ego is stopped (< 0.01 m/s), the desired speed becomes `min(mean waypoint spacing × 4, 0.1)`.
2. **Throttle / brake** come from `LongitudinalLinearRegressionController` (PDM-Lite's controller). It brakes when the desired speed is below 0.05 m/s.
3. **Steering**: `pred_path` is resampled every 0.1 m with a PCHIP spline and sent to `LateralPIDController` (variable look-ahead). If the car is stopped and braking, steering is computed against a straight dummy path.
4. The result is a `carla.VehicleControl(steer ∈ [-1, 1], throttle ∈ [0, 1], brake ∈ [0, 1])`. For the first 40 frames the agent always sends full brake.

---

## 6. Code samples

**One sample → model-ready input** (same logic as `get_input_batch` and `PlanTDataset.__getitem__`):

```python
from plant_variables import PlanTVariables
from dataset import generate_batch

type_nums = PlanTVariables.class_nums
rows = []
for x in boxes:                           # ego-frame boxes, ego removed, filtered as in §2.1
    cls = x["class"]
    if cls in ["car", "walker", "emergency"]:
        rows.append([type_nums[cls], x["position"][0], x["position"][1],
                     rad2deg(x["yaw"]), x["speed"] * 3.6,
                     x["extent"][1] * 2, x["extent"][0] * 2])
    elif cls in type_nums:               # static, static_car, stop_sign, traffic_light
        rows.append([type_nums[cls], x["position"][0], x["position"][1],
                     rad2deg(x["yaw"]), 0.0,
                     x["extent"][1] * 2, x["extent"][0] * 2])

sample = {
    "input": rows,                                    # N x 7
    "output": [],                                     # no forecasting targets at inference
    "route_original": route_xy[:20],                  # 20 x 2, metres, ego frame
    "speed_limit": PlanTVariables.speed_cats[round(speed_limit_mps * 3.6)],
    "BEV": bev_tensor,                                # torch.float32 [3,128,128]
    "ego_speed": ego_speed_mps, "waypoints": 0, "route": [], "target_speed": 0,   # dummies
}
batch = generate_batch([sample])
batch["y_objs"] = None
```

**Run the model and read the outputs:**

```python
import torch
from lit_module import LitHFLM

net = LitHFLM.load_from_checkpoint(ckpt_path, map_location="cpu").eval()
with torch.no_grad():
    _, _, (pred_path, pred_wps, _), attn = net(batch)

pred_wps  = pred_wps[0].numpy()     # (8, 2)  metres, 0.25 s apart
pred_path = pred_path[0].numpy()    # (20, 2) metres
desired_speed = float(((pred_wps[2] - pred_wps[3]) ** 2).sum() ** 0.5 * 4.0)   # m/s
```

**Load a sample record from this folder:**

```python
import json, numpy as np
from PIL import Image
rec = json.load(open("records/02_red_light_stop.json"))
x_objs = np.array(rec["model_input"]["x_objs"]["rows"], dtype=np.float32)   # N x 7
route  = np.array(rec["model_input"]["route_original"]["points_xy_m"])      # 20 x 2
bev    = np.asarray(Image.open("records/" + rec["model_input"]["BEV"]["png"]),
                    dtype=np.float32).transpose(2, 0, 1) / 255           # 3 x 128 x 128 (±0.002)
wps    = np.array(rec["model_output"]["pred_wps"]["points_xy_m"])          # 8 x 2
```

`plant2_pipeline.py` has a numpy-only port of the whole pipeline: object filtering and encoding, quantisation, `interpolate_route`, `generate_batch`, and the desired-speed rule. It runs without torch or CARLA.

---

## 7. Sample records (`records/`)

Seven records. Each one has:
- `<id>.json` — the record: raw boxes and measurements → model input → model output → controller value
- `<id>_bev.png` — the BEV input tensor as an RGB image (divide by 255 for the float tensor; run the script with `--save-npy` for the exact float32 `.npy`)
- `<id>_preview.png` — a picture of the scene

`batch_example.json` shows all seven batched together, as `generate_batch` would produce them.

| # | record | what it shows |
|---|---|---|
| 1 | `01_urban_follow_lead` | Basic case: lead car, oncoming car, follower. 50 km/h (`speed_limit = 0`) |
| 2 | `02_red_light_stop` | Red light as a stop-line box (class 5). A red light on the other approach and a green light are **dropped**. The waypoints shrink as the car brakes to a stop |
| 3 | `03_pedestrian_crossing` | Walker class (2) crossing the lane. A standing pedestrian is kept by the dataset loader but would be dropped by the agent (§8) |
| 4 | `04_construction_detour` | Cones and a warning sign (class 3) with corrected extents. A barbecue prop is dropped. **`route_original` (input) ≠ path target**: the expert detours into the oncoming lane |
| 5 | `05_emergency_vehicle_yield` | An ambulance becomes class 6. 100 km/h (`speed_limit = 2`). The path moves one lane to the right |
| 6 | `06_stop_sign_parked_car` | Stop sign (class 4), a parked `static_car` (id −999, shares class 1), and a car 120 m ahead **outside the range ellipse** (dropped) |
| 7 | `07_highway_cut_in` | 120 km/h (`speed_limit = 3`). A car cutting in, and a car 85 m ahead that is **still in range** thanks to the 100 m front ellipse |

**Where these records come from:** the scenes were built by hand. They are not recordings from a CARLA run, because the dataset on HuggingFace can't be downloaded from the environment where these were made. Every input does go through the ported PlanT 2.0 pipeline, so formats, filtering, units and quantisation are exact. The "outputs" are **expert targets** (what the model is trained to output), not predictions from a trained checkpoint.

**Making records from the real dataset**, after downloading a route folder from [`SimonGer/PlanT2_Dataset`](https://huggingface.co/datasets/SimonGer/PlanT2_Dataset):

```bash
python make_records.py --out records_real \
    --route-dir /path/to/PlanT2_Dataset/data/<scenario>/<route_folder> \
    --frames 20 60 100 --plant2-repo /path/to/plant2
```

This uses `load_real_sample`, a port of `PlanTDataset.__getitem__` without augmentation, and writes records in the same format.

Requirements: `numpy`, `pillow`, `matplotlib`.

---

## 8. Gotchas

- **The dataset and the agent filter slightly differently.**
  - Stationary pedestrians: the agent drops them until they first move; the dataset loader keeps them.
  - Traffic lights: the dataset checks the box's `affects_ego` flag; the agent picks the next light along the route itself.
  - `static` objects are removed entirely on `longest6` routes.
- **The speed limit must be 50/80/100/120 km/h.** Any other value raises a `KeyError` in `speed_cats`.
- **`augment_parked: True`** loads `car_data.npy` from a hard-coded cluster path (`/home/geiger/...`) in `dataset.py`. Change it before training locally.
- **Augmentation** (50 % of samples): shifts and rotates the scene using `augmentation_translation` / `augmentation_rotation`, so the model learns to recover from drifting off the route. The BEV then switches to `bev_no_car_semantics_augmented`.
- **Data filtering:** the loader skips routes where the expert scored < 100 (minimum-speed infractions excepted). It also skips the first 5 and last ~10 frames of each route.
