
#!/usr/bin/env python3

"""
run_lio_sam.py

Unified entry point for running ROS 2 bags through LIO-SAM.

This script automates the LIO-SAM mapping pipeline, from launching the
required ROS 2 nodes to processing a bag and exporting the pcd and tum.

Features:
- Accepts a ROS 2 bag path, parameter file, playback rate, and output directory.
- Supports configurable ROS 2 launch packages and launch files.
- Launches and monitors LIO-SAM during bag playback.
- Checks odometry health during processing.
- Saves the accumulated point cloud map as PCD files.
- Records log in the specified output directory as lio_sam.log.
- Exports the live and optimized LIO-SAM trajectories in TUM format.
- Optionally records and exports ground-truth odometry in TUM format.
- Cleans up launched processes and ROS 2 nodes after execution.

Usage:
python3 scripts/run_lio_sam.py --help

Example:
python3 scripts/run_lio_sam.py \
    --bag <path>/paper_dataset \
    --params /config/params.yaml \
    --rate 0.5 \
    --output <path>/results

Example #2 (for CARLA filtered bags):
python3 scripts/run_lio_sam.py \
    --bag <path>/carla_filtered \
    --params /config/params.yaml \
    --rate 0.5 \
    --output <path>/results
    --ground-truth

Requirements:
- ROS 2 Jazzy
- LIO-SAM built in the AP1 workspace
- A compatible ROS 2 bag containing LiDAR and IMU data
- A corresponding LIO-SAM parameter file

Output:
- PCD map files
- LIO-SAM live trajectory
- LIO-SAM optimized trajectory
- Optional ground-truth trajectory (add --ground-truth in args)
- LIO-SAM execution log (should be in output directory)
"""

import argparse
import sys
from pathlib import Path
import subprocess
import shlex
import shutil
import time
import os
import signal

import threading
import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path as RosPath

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")

# AP1 workspace
AP1_WORKSPACE = Path.home() / "WEAP/ap1"
WORKSPACE_SETUP = AP1_WORKSPACE / "install/setup.bash"

class OdometryMonitor:

    def __init__(self):

        self.node = Node("lio_sam_health_check")

        self.last_odom_time = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.node.create_subscription(
            Odometry,
            "/lio_sam/mapping/odometry",
            self.odometry_callback,
            qos
        )

        # gives each node its own executor, keeping the same ROS context
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )

        self.spin_thread.start()

    def odometry_callback(self, msg):
        self.last_odom_time = time.monotonic()

    def check_health(self, timeout=30):
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:

            if self.last_odom_time is not None:
                print("[INFO] LIO-SAM is publishing odometry.")
                return True

            time.sleep(0.1)

        print("[ERROR] LIO-SAM did not publish odometry within timeout.")
        return False

    def wait_for_idle(self, idle_timeout=3.0, max_wait=60.0):
        start_time = time.monotonic()

        if self.last_odom_time is None:
            print("[ERROR] No odometry received.")
            return False

        while time.monotonic() - start_time < max_wait:

            idle_time = time.monotonic() - self.last_odom_time

            if idle_time >= idle_timeout:
                print(
                    f"[INFO] LIO-SAM has been idle for "
                    f"{idle_time:.1f} seconds."
                )
                return True

            time.sleep(0.1)

        print("[ERROR] LIO-SAM did not become idle within timeout.")
        return False

    def shutdown(self):

        self.executor.shutdown(timeout_sec=5)

        self.spin_thread.join(timeout=5)

        self.executor.remove_node(self.node)
        self.node.destroy_node()

class TrajectoryRecorder(Node):

    def __init__(self, record_ground_truth=False):
        super().__init__("trajectory_recorder")

        self.lio_trajectory = []
        self.gt_trajectory = []

        self.lock = threading.Lock()

        # Subscribe to LIO-SAM odometry
        self.lio_subscription = self.create_subscription(
            Odometry,
            "/lio_sam/mapping/odometry",
            self.lio_callback,
            qos
        )

        # Store the latest loop-closure-optimized trajectory
        self.optimized_trajectory = []

        # Subscribe to path
        self.path_subscription = self.create_subscription(
            RosPath,
            "/lio_sam/mapping/path",
            self.path_callback,
            qos
        )

        # Optionally subscribe to CARLA ground truth
        self.gt_subscription = None

        if record_ground_truth:
            self.gt_subscription = self.create_subscription(
                Odometry,
                "/ground_truth/odometry",
                self.gt_callback,
                qos
            )

        self.get_logger().info(
            "Trajectory recorder initialized."
        )

    def extract_pose(self, msg):
        """Extract timestamp, position and orientation."""

        stamp = msg.header.stamp

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Preserve the original ROS timestamp precision
        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"

        return [
            timestamp,
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]

    def lio_callback(self, msg):
        with self.lock:
            self.lio_trajectory.append(
                self.extract_pose(msg)
            )

    def gt_callback(self, msg):
        with self.lock:
            self.gt_trajectory.append(
                self.extract_pose(msg)
            )
            
    def path_callback(self, msg):
        """Store the latest optimized LIO-SAM path."""

        trajectory = []

        for pose_stamped in msg.poses:
            stamp = pose_stamped.header.stamp
            pose = pose_stamped.pose

            timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"

            trajectory.append([
                timestamp,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

        with self.lock:
            self.optimized_trajectory = trajectory

    def save_trajectory(self, trajectory, output_file):
        """Save a trajectory in TUM format."""

        if not trajectory:
            raise RuntimeError(
                f"No trajectory data received for {output_file.name}"
            )

        with open(output_file, "w") as file:

            for pose in trajectory:

                timestamp = pose[0]

                values = " ".join(
                    f"{value:.9f}" for value in pose[1:]
                )

                file.write(f"{timestamp} {values}\n")

        print(
            f"Saved {len(trajectory)} poses to {output_file}"
        )

    def export_trajectories(self, output_dir, record_ground_truth=False):

        with self.lock:
            lio_data = list(self.lio_trajectory)
            optimized_data = list(self.optimized_trajectory)
            gt_data = list(self.gt_trajectory)

        # Export live LIO-SAM odometry
        lio_file = output_dir / "lio_sam_trajectory.tum"
        self.save_trajectory(lio_data, lio_file)

        # Export optimized LIO-SAM path
        optimized_file = output_dir / "lio_sam_optimized_trajectory.tum"
        self.save_trajectory(optimized_data, optimized_file)

        # Export ground truth if processing a CARLA bag
        if record_ground_truth:
            gt_file = output_dir / "ground_truth_trajectory.tum"
            self.save_trajectory(gt_data, gt_file)

        print("Trajectory export completed.")

def start_trajectory_recorder(record_ground_truth=False):

    if not rclpy.ok():
        rclpy.init()

    recorder = TrajectoryRecorder(
        record_ground_truth=record_ground_truth
    )

    executor = SingleThreadedExecutor()
    executor.add_node(recorder)

    recorder_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    recorder_thread.start()

    return recorder, executor, recorder_thread


def stop_trajectory_recorder(
    recorder,
    executor,
    recorder_thread
):

    executor.shutdown(timeout_sec=5)

    recorder_thread.join(timeout=5)

    executor.remove_node(recorder)
    recorder.destroy_node()

def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run LIO-SAM on a ROS 2 bag."
    )

    parser.add_argument(
        "--bag",
        required=True,
        help="Path to the ROS 2 bag directory."
    )

    parser.add_argument(
        "--params",
        required=True,
        help="Path to the LIO-SAM parameter YAML file."
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=0.5,
        help="Bag playback rate (default: 0.5)."
    )

    parser.add_argument(
        "--output",
        required=True,
        help="Directory where results will be saved."
    )

    parser.add_argument(
        "--ground-truth",
        action="store_true",
        help="Also record CARLA ground-truth odometry."
    )

    parser.add_argument(
        "--launch-pkg",
        default="lio_sam",
        help="ROS 2 package containing the launch file"
    )

    parser.add_argument(
        "--launch-file",
        default="run.launch.py",
        help="ROS 2 launch file to execute"
    )

    return parser.parse_args()


def validate_inputs(args):
    bag_path = Path(args.bag)
    params_path = Path(args.params)

    # Check that the bag exists
    if not bag_path.exists() or not bag_path.is_dir():
        raise FileNotFoundError(f"Bag directory not found: {bag_path}")

    # Check that the parameter file exists
    if not params_path.exists() or not params_path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {params_path}")

    # Validate playback rate
    if args.rate <= 0:
        raise ValueError("Playback rate must be greater than 0.")

    return bag_path.resolve(), params_path.resolve()


def create_output_directory(output_path):
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    return output_dir.resolve()


def start_pipeline(
    bag_path,
    params_path,
    rate,
    output_dir,
    launch_pkg,
    launch_file,
    record_ground_truth=False,
):

    print("Starting LIO-SAM pipeline...")

    # Task 2: Launch LIO-SAM
    lio_process = launch_lio_sam(
        params_path,
        launch_pkg,
        launch_file,
        output_dir
    )

    if lio_process.poll() is not None:
        raise RuntimeError(
            f"LIO-SAM exited unexpectedly (Task 2: while launching LIO-SAM): "
            f"{lio_process.returncode}"
        )

    print("LIO-SAM is running.")

    # Task 5: Start trajectory recording BEFORE playback
    recorder, recorder_executor, recorder_thread = (
        start_trajectory_recorder(record_ground_truth)
    )

    odom_monitor = None

    try:
        odom_monitor = OdometryMonitor()

        # Task 3: Play bag
        bag_process = play_bag(
            bag_path,
            rate,
            lio_process,
            odom_monitor
        )

        print("Bag playback completed.")

        # Check LIO-SAM is still running
        if lio_process.poll() is not None:
            raise RuntimeError(
                f"LIO-SAM exited unexpectedly (Task 3: While playing bag): "
                f"{lio_process.returncode}"
            )

        # Wait for final odometry messages
        if not odom_monitor.wait_for_idle(
            idle_timeout=3.0,
            max_wait=60.0
        ):
            raise RuntimeError(
                "LIO-SAM did not finish processing in time."
            )

        # Task 4: Save map
        saved_maps = save_map(output_dir)

        # Task 5: Export trajectories
        recorder.export_trajectories(
            output_dir,
            record_ground_truth
        )

        return lio_process, bag_process, saved_maps

    finally:
        # cleanup
        if odom_monitor is not None:
            odom_monitor.shutdown()

        stop_trajectory_recorder(
            recorder,
            recorder_executor,
            recorder_thread
        )

        if rclpy.ok():
            rclpy.shutdown()

        os.killpg(lio_process.pid, signal.SIGINT)
        
def launch_lio_sam(params_path, launch_pkg, launch_file, output_dir):
    """
    Launch LIO-SAM using the supplied parameter file.

    Saves LIO-SAM output to lio_sam.log.
    Returns the subprocess.Popen object.
    """

    # Check ROS 2 setup files
    if not ROS_SETUP.is_file():
        raise FileNotFoundError(
            f"ROS 2 setup file not found: {ROS_SETUP}"
        )

    if not WORKSPACE_SETUP.is_file():
        raise FileNotFoundError(
            f"Workspace setup file not found: {WORKSPACE_SETUP}"
        )

    print("Launching LIO-SAM...")

    # Create output directory if necessary
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Log file location
    log_path = output_dir / "lio_sam.log"

    # Construct the ROS 2 launch command
    command = (
        f"set -e; "
        f"source {shlex.quote(str(ROS_SETUP))}; "
        f"source {shlex.quote(str(WORKSPACE_SETUP))}; "
        f"exec ros2 launch "
        f"{shlex.quote(launch_pkg)} "
        f"{shlex.quote(launch_file)} "
        f"params_file:={shlex.quote(str(params_path))}"
    )

    # Start LIO-SAM as a separate process
    with open(log_path, "w") as log_file:
        process = subprocess.Popen(
            ["bash", "-c", command],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            cwd=AP1_WORKSPACE
        )

    print(f"LIO-SAM started (PID: {process.pid})")
    print(f"LIO-SAM log: {log_path}")

    return process


def play_bag(bag_path, rate, lio_process, odom_monitor):

    print("Waiting for LIO-SAM to initialize...")

    time.sleep(5)

    if lio_process.poll() is not None:
        raise RuntimeError(
            f"LIO-SAM exited before bag playback. "
            f"Exit code: {lio_process.returncode}"
        )

    print("Starting ROS 2 bag playback...")

    command = [
        "ros2",
        "bag",
        "play",
        str(bag_path),
        "--clock",
        "--rate",
        str(rate)
    ]

    bag_process = subprocess.Popen(command)

    print(f"Bag playback started (PID: {bag_process.pid})")

    # Verify LIO-SAM starts publishing odometry
    if not odom_monitor.check_health(timeout=30):
        bag_process.terminate()
        bag_process.wait()
        raise RuntimeError("LIO-SAM failed its health check.")
    print("[DEBUG] Health check passed. Waiting for bag completion...")

    # Wait for playback to finish
    return_code = bag_process.wait()

    if return_code != 0:
        raise RuntimeError(
            f"Bag playback failed with exit code {return_code}"
        )

    print("Bag playback completed successfully.")

    return bag_process



def save_map(output_dir, resolution=0.2):
    """
    Request LIO-SAM to save its accumulated point cloud map
    inside the WSL filesystem.
    """

    output_dir = Path(output_dir).expanduser().resolve()
    map_dir = output_dir / "map"

    # LIO-SAM concatenates HOME directly with destination.
    destination = "/" + os.path.relpath(map_dir, Path.home()).lstrip("/")

    saved_map_dir = Path(str(Path.home()) + destination)

    request = (
        f"{{resolution: {resolution}, "
        f"destination: '{destination}'}}"
    )

    print("Requesting LIO-SAM to save map...")
    print(f"Output directory: {map_dir}")
    print(f"LIO-SAM save directory: {saved_map_dir}")

    command = (
        f"source {shlex.quote(str(ROS_SETUP))} && "
        f"source {shlex.quote(str(WORKSPACE_SETUP))} && "
        f"ros2 service call /lio_sam/save_map "
        f"lio_sam/srv/SaveMap "
        f"{shlex.quote(request)}"
    )

    try:
        result = subprocess.run(
            ["bash", "-c", command],
            capture_output=True,
            text=True,
            timeout=120
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError("Map saving timed out.")

    if result.returncode != 0:
        raise RuntimeError(
            f"Map saving failed:\n{result.stderr}"
        )

    print(result.stdout)

    pcd_files = list(saved_map_dir.glob("*.pcd"))

    if not pcd_files:
        raise RuntimeError(
            f"No PCD files generated in {saved_map_dir}"
        )

    print("\nMap files generated successfully:")

    for pcd_file in pcd_files:
        print(f"Generated: {pcd_file}")

    return pcd_files



def main():
    try:
        
        args = parse_arguments()

        bag_path, params_path = validate_inputs(args)

        output_dir = create_output_directory(args.output)

        start_pipeline(
            bag_path,
            params_path,
            args.rate,
            output_dir,
            args.launch_pkg,
            args.launch_file,
            record_ground_truth=args.ground_truth,
        )

    except (FileNotFoundError, ValueError, OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
