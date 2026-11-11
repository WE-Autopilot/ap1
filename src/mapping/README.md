# Mapping (5.3): CARLA map in one command

`scripts/map_carla.sh` runs the whole mapping pipeline:

```
CARLA drive ─record_bag.py─> raw bag ─filter_bag.py─> filtered bag ─run_lio_sam.py─> map + trajectories + summary
```

The filtered bag carries the same ICD topics the kart will publish
(`/sensing/lidar/pointcloud`, `/sensing/imu/data`, `/tf_static`), so everything
after the filter is the same pipeline we'll run on real kart bags.

## Before the first run (once per machine)

1. Build the workspace with LIO-SAM ([SETUP.md](../../SETUP.md)):
   ```bash
   ./ap1_setup.sh
   ```
   It imports LIO-SAM, patches it for Jazzy, installs dependencies and runs
   `colcon build`. After that, rebuild with plain `colcon build` from the repo
   root. Release is already set in `colcon.meta`; don't add `--symlink-install`
   (mixing build styles breaks `build/`).
2. Install the CARLA client and the recorder's packages:
   ```bash
   python3 -m pip install -r simulation/requirements.txt --break-system-packages
   ```
3. For checking the map: `pip install evo --break-system-packages` (error vs
   ground truth), `sudo apt install pcl-tools` (`pcl_viewer`), and optionally
   [CloudCompare](https://www.cloudcompare.org/) on Windows.
4. CARLA on Windows, scripts in WSL: turn on mirrored networking. See
   [simulation/README.md](../../simulation/README.md), "Recording a raw bag".

## Start CARLA

One CARLA instance (0.9.16) is enough; the script loads the map you ask for
inside it (the window freezes for up to a minute while it loads).

```
CarlaUE4.exe                     # with a window: the view follows the car from above
CarlaUE4.exe -RenderOffScreen    # no window: lighter, once a town is known to work
```

Keep the window for the first run on a new town so you can see the car get
stuck or drive somewhere unexpected. Ports 2000 and 2001 must be reachable.

## Run it

From the repo root:

```bash
scripts/map_carla.sh --map Town01 --short
```

The script checks your setup, records a closed loop, filters it to the kart's
sensors, runs LIO-SAM at 0.5×, saves the map and prints a summary. Drop
`--short` to drive every road of the map (much longer, much bigger bags).

To map a bag you already recorded (CARLA not needed):

```bash
scripts/map_carla.sh --bag ~/WEAP/bags/<name>_filtered
```

`scripts/map_carla.sh --dry-run ...` prints the commands without running them.

**Towns:** Town01 works. **Town02 crashes CARLA 0.9.16** at the same spot every
time (by the plaza, about 47% of the `--short` loop) with "Fatal error" or "Pure
virtual function being called". It's a CARLA bug, not your machine: use
Town01, Town03 or Town05. Open highway towns (Town04/06) give the LiDAR little
to match.

## What you get

Each run gets its own folder, `~/WEAP/maps/<date>_<time>_<map>/`:

| File | What it is |
|---|---|
| `results/map/GlobalMap.pcd` | the map (0.2 m voxels) |
| `results/map/CornerMap.pcd`, `SurfMap.pcd` | LIO-SAM's edge and surface feature maps |
| `results/map/trajectory.pcd` | keyframe positions; `intensity` is the keyframe number (0 = start) |
| `results/lio_sam_optimized_trajectory.tum` | LIO-SAM's path after loop closure |
| `results/lio_sam_trajectory.tum` | LIO-SAM's live odometry, before loop closure |
| `results/ground_truth_trajectory.tum` | CARLA's true path |
| `results/ape.txt` | error vs ground truth (only if `evo` is installed) |
| `results/lio_sam.log` | LIO-SAM's output |
| `lio_sam_params.yaml` | the exact params this map was made with |
| `bag_raw/`, `bag_filtered/` | the recorded bags (only with `--map`) |
| `run.log` | everything the script printed |

The `.tum` files are paths as text, one pose per line
(`time x y z qx qy qz qw`). They don't make the map; they're how you measure it.

## Is the map good?

**Reference run** (Town01 `--short`, 0.73 km, CV7 IMU values, 2026-11-11):

| Check | Result |
|---|---|
| LIO-SAM path vs ground truth length | 723.2 m vs 724.0 m |
| Error after loop closure (evo RMSE) | 0.33 m (max 0.58 m) |
| Error before loop closure (live odometry) | 1.68 m (max 3.46 m) |
| Height difference, first vs last keyframe | 3.4 cm |
| GlobalMap.pcd size | 39 MB |

**1. Error vs ground truth (the number that matters).** In `results/`:

```bash
evo_ape tum ground_truth_trajectory.tum lio_sam_optimized_trajectory.tum --align --t_max_diff 0.05 --save_results opt.zip
evo_ape tum ground_truth_trajectory.tum lio_sam_trajectory.tum --align --t_max_diff 0.05 --save_results live.zip
evo_res live.zip opt.zip
```

`rmse` is the headline. The optimized path should be clearly lower than the
live one; if not, loop closure didn't help. To see it, add
`-p --plot_mode xy` (path colored by error), or overlay all three paths:

```bash
evo_traj tum lio_sam_trajectory.tum lio_sam_optimized_trajectory.tum \
  --ref ground_truth_trajectory.tum --align -p --plot_mode xy
```

No plot window in WSL? `evo_config set plot_backend Agg`, add
`--save_plot plot.png`, then `explorer.exe .` to open it from Windows.

**The summary's "end-to-end error" is only a rough check.** The car stops up to
a metre from where it started, and LIO-SAM adds a keyframe only every 1 m, so
about 1 m there is normal. Trust evo.

**2. Look at the map.**

- Quick look: `pcl_viewer results/map/GlobalMap.pcd` (`1`–`5` colors, `+`/`-` point size, `r` reset).
- Closer check in CloudCompare: open `GlobalMap.pcd` (from Windows:
  `\\wsl$\Ubuntu\root\WEAP\maps\...`; `wsl -l` gives the distro name).
  1. Find the start: also open `map/trajectory.pcd`, set Colors → Scalar field →
     `intensity`, point size 5. Blue is the start (at 0, 0, 0), red is the end.
  2. Look for doubled walls: Tools → Segmentation → Cross Section. Set a thin box
     (e.g. X 30, Y 2, full Z) across the start/end area and view it from the side.
     A good map has one ground line and single outlines; the same shape twice,
     offset, means the loop didn't fully close there.
  3. Measure anything with Tools → Point picking.

## Running LIO-SAM by hand

For tuning or debugging, run the pieces yourself and watch in RViz:

```bash
ros2 launch ap1_mapping lio_sam_carla.launch.py          # LIO-SAM + RViz; params_file:=<yaml> rviz:=false
ros2 bag play <run>/bag_filtered --clock --rate 0.5       # second terminal; space bar pauses
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/WEAP/manual_map'}"
```

`destination` is relative to your home folder, and LIO-SAM deletes that folder
before writing, so use a new one each time.

## Options

| Option | Default | Use it when |
|---|---|---|
| `--map <Town>` | | recording a new drive |
| `--short` | off | you want the quick loop, not every road |
| `--spawn-point N` | 0 | the route fails ("can't get back to the start", "inside a junction"); check the printed route changes |
| `--bag <path>` | | re-mapping a filtered bag you already have |
| `--paper-imu` | off | LIO-SAM's IMU node dies or the path flies off (uses the paper's IMU noise instead of the CV7 values) |
| `--rate R` | 0.5 | slower if LIO-SAM falls behind |
| `--host <host>` | `$CARLA_HOST` or `localhost` | CARLA runs on another machine (e.g. a desktop when a laptop struggles) |
| `--params <yaml>` | `src/mapping/config/lio_sam_carla.yaml` | trying other LIO-SAM params |
| `--out <dir>` | `~/WEAP/maps` | saving runs somewhere else |

## When it goes wrong

| Symptom | Fix |
|---|---|
| `workspace not built` / `lio_sam isn't built` | Do "Before the first run" above |
| Route fails before driving | Try another `--spawn-point`; check the printed route length changes (nearby spawn points can give the same loop) |
| CARLA "Fatal error" / "Pure virtual function being called" mid-drive | Town02 bug (see Towns). Use another town. Crash dumps: `%LOCALAPPDATA%\CarlaUE4\Saved\Crashes` |
| `time-out ... waiting for the simulator` | CARLA crashed or froze. Restart CARLA (it may be stuck in synchronous mode) |
| Recording stopped early ("is partial") | The bag is still usable for testing (no loop closure): `python3 simulation/filter_bag.py <raw> <filtered>`, then `--bag <filtered>` |
| `LIO-SAM run FAILED`, `imuPreintegration` died, or `Large velocity` in the log | Rerun the same bag with `--paper-imu --bag <run folder>/bag_filtered` |
| `mapOptimization ... escalating to 'SIGTERM'` in `lio_sam.log` | Harmless: LIO-SAM's own save on shutdown (`savePCD: true`) was slow. The map was already saved |
| Two paths in RViz, or an old run interfering | `pkill -f lio_sam_`, then check `ros2 node list` is empty |
| Large evo error or a doubled map | Loop closure missed. Use a town with buildings |
| Low disk warning | Raw bags are big; free space or use `--short` |

Don't keep anything in `~/ap1_maps/lio_sam_carla/`: LIO-SAM deletes it on every
shutdown. The map that matters is in the run folder.