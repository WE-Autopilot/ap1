# Mapping (5.3): CARLA map in one command

`scripts/map_carla.sh` runs the whole mapping pipeline:

```
CARLA drive ─record_bag.py─> raw bag ─filter_bag.py─> filtered bag ─run_lio_sam.py─> map + trajectories + summary
```

## Before the first run (once per machine)

1. Build the workspace with LIO-SAM ([SETUP.md](../../SETUP.md), "LIO-SAM (map building)"):
   ```bash
   ./ap1_setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
2. Install the CARLA client and the recorder's packages:
   ```bash
   python3 -m pip install -r simulation/requirements.txt --break-system-packages
   ```
3. Optional, for an RMSE against ground truth: `pip install evo --break-system-packages`
4. CARLA on Windows, scripts in WSL: turn on mirrored networking. See
   [simulation/README.md](../../simulation/README.md), "Recording a raw bag".

## Run it

Start CARLA, then from the repo root:

```bash
scripts/map_carla.sh --map Town02 --short
```

That's all. The script checks your setup, records a closed loop, filters it to
the kart's sensors, runs LIO-SAM at 0.5×, saves the map and prints a summary.
Drop `--short` to drive every road of the map (much longer).

To map a bag you already recorded (CARLA not needed):

```bash
scripts/map_carla.sh --bag ~/WEAP/bags/<name>_filtered
```

`scripts/map_carla.sh --dry-run ...` prints the commands without running them.

## What you get

Each run gets its own folder, `~/WEAP/maps/<date>_<time>_<map>/`:

| File | What it is |
|---|---|
| `results/map/GlobalMap.pcd` | the map (0.2 m voxels). Open in CloudCompare. |
| `results/map/CornerMap.pcd`, `SurfMap.pcd` | LIO-SAM's edge and surface feature maps |
| `results/lio_sam_optimized_trajectory.tum` | LIO-SAM's path after loop closure |
| `results/lio_sam_trajectory.tum` | LIO-SAM's live odometry |
| `results/ground_truth_trajectory.tum` | CARLA's true path |
| `results/ape.txt` | RMSE vs ground truth (only if `evo` is installed) |
| `results/lio_sam.log` | LIO-SAM's output |
| `lio_sam_params.yaml` | the exact params this map was made with |
| `bag_raw/`, `bag_filtered/` | the recorded bags (only with `--map`) |
| `run.log` | everything the script printed |

The summary prints the **end-to-end error**: the distance between where
LIO-SAM's loop starts and ends. The car finishes where it started, so this
should be near 0.

## Options

| Option | Default | Use it when |
|---|---|---|
| `--map <Town>` | | recording a new drive |
| `--short` | off | you want the quick loop, not every road |
| `--spawn-point N` | 0 | the route fails ("can't get back to the start", "inside a junction") |
| `--bag <path>` | | re-mapping a filtered bag you already have |
| `--paper-imu` | off | LIO-SAM's IMU node dies or the path flies off (uses the paper's IMU noise instead of the CV7 values) |
| `--rate R` | 0.5 | slower if LIO-SAM falls behind |
| `--host <host>` | `$CARLA_HOST` or `localhost` | CARLA runs on another machine |
| `--params <yaml>` | `src/mapping/config/lio_sam_carla.yaml` | trying other LIO-SAM params |
| `--out <dir>` | `~/WEAP/maps` | saving runs somewhere else |

## When it goes wrong

| Symptom | Fix |
|---|---|
| `workspace not built` / `lio_sam isn't built` | Do "Before the first run" above |
| Route fails before driving | Try another `--spawn-point` |
| `LIO-SAM run FAILED`, `imuPreintegration` died, or `Large velocity` in the log | Rerun the same bag with `--paper-imu --bag <run folder>/bag_filtered` |
| End-to-end error of metres, or a doubled map | Loop closure missed. Use a town with buildings (open highway towns like Town04/06 have little for the LiDAR to match) |
| Low disk warning | Raw bags are big; free space or use `--short` |

Pick a town with buildings and walls for the first try. Only the Town01 loop has
been mapped so far, so the first run on a new map is the real test of the params.
