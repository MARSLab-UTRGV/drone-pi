# Drone Companion Computer — System Overview

This document describes the companion-computer ROS 2 stack running on a Raspberry Pi 5 used together with ArduPilot (Pixhawk) as the flight controller. The goal of the stack is to detect AprilTags from a USB camera, publish tag poses into TF, and provide a simple hover + yaw-search controller that can (eventually) lock onto tags. The Pi records and streams imagery for later vibration analysis once the vehicle is placed into a stable pose.

## High-level purpose

- Provide a ROS 2-based perception and control pipeline on the Pi that: captures images, detects AprilTags, produces stable tag poses in TF, and issues velocity setpoints to the Pixhawk (via MAVROS) for yaw-search and eventual tag lock.
- Intended use-case: place drone near structure, perform an AprilTag-guided yaw-search and then hold position while recording video for vibration/structural analysis.

## Important components (nodes/scripts)

- Camera node (V4L2-based)
  - Generic USB camera treated as a V4L2 device (example device `/dev/video4`).
  - Typical parameters used in this project: `video_device`, `image_size` (e.g. `[1280,720]`), `pixel_format` (e.g. `YUYV`), `output_encoding` (e.g. `rgb8`), `frame_id` (e.g. `camera`), and `camera_info_url` that points to a stored calibration YAML.
  - Publishes: `/image_raw` (sensor_msgs/Image), `/camera_info` (sensor_msgs/CameraInfo)

- apriltag_ros
  - AprilTag detector node that subscribes to an image topic (rectified image) and camera info, runs detection, and publishes detections on `/detections` (apriltag_msgs/AprilTagDetectionArray).
  - Uses a parameters file (`~/apriltag_params.yaml`) in this project. Example parameters include `family: 36h11`, `tag_size: 0.0376`, `publish_tf: true` (but we prefer using a PnP broadcaster for pose stability).

- AprilTag PnP TF broadcaster (`~/apriltag_pnp_broadcaster.py`)
  - Standalone Python node included in the Pi home directory.
  - Subscribes to `/camera_info` and `/detections` (AprilTag detections). Uses camera intrinsics and detected 2D corner pixels to run OpenCV `solvePnP` (IPPE / ITERATIVE fallback) to compute tag pose relative to camera.
  - Publishes TF frames using `tf2_ros.TransformBroadcaster`, child frames named `tag36h11:<id>` (e.g. `tag36h11:0`) with parent frame set to the camera frame (default `camera`).
  - Parameters supported (via ROS params or local defaults): `camera_frame`, `tag_prefix`, `tag_size_m`, `detections_topic`, `camera_info_topic`.
  - Notes: This PnP approach uses the four tag corners and is preferred when you need an explicit tag pose and better control over the pose estimation pipeline.

- Alternative TF broadcaster (`~/apriltag_tf_broadcaster.py`)
  - A more generic broadcaster that expects the detection message to contain a pose field from the detector package. It tolerates a few different message layouts and emits TF frames named `tag36h11:<id>`.
  - Useful when the detector already produces poses (PoseStamped or PoseWithCovarianceStamped). Use when you want a lightweight relay of pose->TF.

- Visualization overlay (`~/tag_overlay.py`)
  - Subscribes to `/image_raw` and `/detections` and publishes `/image_with_tags` with polylines and tag id labels for quick visual debugging (uses cv_bridge and OpenCV). Useful when running `image_view` or a simple image topic viewer.

- MAVROS node (mavros_node)
  - Connects to Pixhawk (FCU) over serial (example: `fcu_url:=serial:///dev/ttyAMA0:57600`).
  - Publishes/subscribes many topics; relevant here are:
    - SUB: `/mavros/state` (mavros_msgs/State) — used by the hover controller for safety/mode checks
    - PUB: `/mavros/setpoint_velocity/cmd_vel_unstamped` (geometry_msgs/Twist) — velocity setpoint topic used to command yaw-rate and small body-frame velocities.

- Hover + yaw-search controller (`tag_hover_controller/hover_yaw_search.py`)
  - ROS 2 Python node packaged in `~/ros2_ws/src/tag_hover_controller`.
  - Behavior:
    - SEARCH mode: publish a constant yaw rate until a tag TF becomes available.
    - LOCK mode: query TF (`camera` → `tag36h11:0`), compute yaw error using tag position in camera frame (yaw_error ≈ atan2(x, z)), then command an angular.z yaw rate using a P controller (clamped by `max_yaw_rate`).
    - Currently implements SEARCH mode (constant yaw). LOCK mode and P-controller on tag yaw are planned and partially scaffolded in the code.
  - Subscriptions/Pubs:
    - SUB: `/mavros/state` (mavros_msgs/State)
    - PUB (debug): `/hover_yaw_cmd` (geometry_msgs/Twist)
    - PUB (to FCU): `/mavros/setpoint_velocity/cmd_vel_unstamped` (geometry_msgs/Twist)
  - Key parameters defined in the node:
    - `mode` (`SEARCH` or `LOCK`)
    - `rate_hz` (control loop rate)
    - `search_yaw` (rad/s constant yaw rate in SEARCH)
    - `lock_k_yaw` (P gain for yaw control)
    - `camera_frame`, `tag_frame` (frame names used for lookup)
    - `max_yaw_rate` (clamp on commanded yaw rate)

## Data flow (diagram-style, text)

- camera (V4L2) → publishes `/image_raw` and `/camera_info`.
- `/image_raw` → apriltag_ros → publishes `/detections` (AprilTagDetectionArray).
- `/camera_info` + `/detections` → apriltag_pnp_broadcaster → publishes TF frames into `/tf`: `camera` → `tag36h11:<id>`.
- `hover_yaw_search` → listens to `/mavros/state` and TF (`camera` → `tag36h11:0`). In SEARCH mode it publishes constant yaw rates; in LOCK mode it uses TF to compute yaw error and publishes a yaw-rate command to `/mavros/setpoint_velocity/cmd_vel_unstamped` → MAVROS → Pixhawk.
- `tag_overlay` (visualization) subscribes to `/image_raw` and `/detections`, publishes `/image_with_tags` for viewing.

## How to run this stack (step-by-step examples)

Notes:
- The exact launch/run commands depend on which ROS 2 packages you have installed on the Pi. The examples below are explicit commands you can adapt. Replace paths and device names as needed.
- Always verify MAVROS connection, and perform flight tests with the vehicle disarmed or props removed for safety.

1) Launch the camera node (example using a V4L2 camera node):

```bash
# example: using a generic v4l2 camera node (adjust to the package/exe you have)
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -p video_device:=/dev/video4 \
             -p "image_size:='[1280,720]" \
             -p time_per_frame.num:=1 -p time_per_frame.den:=30 \
             -p pixel_format:=YUYV \
             -p output_encoding:=rgb8 \
             -p frame_id:=camera \
             -p camera_info_url:=file:///home/mars/.ros/camera_info/camera_1280x720.yaml
```

Or, if you have a launch file for the camera, use `ros2 launch <camera_pkg> <camera_launch>.py` with the same parameters.

2) Launch AprilTag detection (example using `apriltag_ros`):

```bash
# example CLI run (adjust to actual executable if different)
ros2 run apriltag_ros apriltag_node \
  --ros-args -p image_rect:=/image_raw -p camera_info:=/camera_info \
             --params-file /home/mars/apriltag_params.yaml
```

3) Launch the PnP TF broadcaster (two ways):

- Run as a standalone script (quick):
```bash
python3 ~/apriltag_pnp_broadcaster.py
```
- Or, if packaged as a ROS 2 node / installed entry point, use:
```bash
ros2 run <package> apriltag_pnp_broadcaster --ros-args \
  -p camera_frame:=camera -p tag_prefix:=tag36h11 -p tag_size_m:=0.0376 \
  -p detections_topic:=/detections -p camera_info_topic:=/camera_info
```

4) (Optional) If you prefer the detector-provided poses relayed to TF, run the alternative broadcaster:

```bash
python3 ~/apriltag_tf_broadcaster.py
```

5) Launch the image overlay for visual debugging:

```bash
python3 ~/tag_overlay.py
# view with image_view or rqt_image_view:
ros2 run image_tools showimage --ros-args -r image:=/image_with_tags
```

6) Launch RViz2 to visualize TF and camera image (optional):

```bash
ros2 run rviz2 rviz2
# In RViz, add TF and Image displays; set the Image topic to /image_with_tags or /image_raw
```

7) Launch MAVROS and the hover controller (example):

```bash
# MAVROS (example using a mavros launch file; adjust to your installed package)
ros2 launch mavros apm.launch.py fcu_url:=serial:///dev/ttyAMA0:57600

# Launch the hover_yaw_search controller via its package launch
ros2 launch tag_hover_controller lockon_backbone.launch.py

# Or run the node directly (if the package is sourced/installed):
ros2 run tag_hover_controller hover_yaw_search --ros-args -p mode:=SEARCH -p rate_hz:=20.0
```

Important: ensure MAVROS reports a connection on `/mavros/state` and that the FCU is in a safe/testing mode such as GUIDED, GUIDED_NOGPS, or LOITER before commanding velocities.

## Parameters of interest (quick reference)

- PnP broadcaster (defaults shown where available):
  - `camera_frame` (default `camera`)
  - `tag_prefix` (default `tag36h11`)
  - `tag_size_m` (set to tag physical size, e.g. `0.0376`)
  - `detections_topic` (default `/detections`)
  - `camera_info_topic` (default `/camera_info`)

- hover_yaw_search (from node source):
  - `mode` (`SEARCH` / `LOCK`)
  - `rate_hz` (control loop rate, default 20.0)
  - `search_yaw` (rad/s, default 0.25)
  - `lock_k_yaw` (P gain for yaw, default 1.0)
  - `camera_frame` (default `camera`)
  - `tag_frame` (default `tag36h11:0`)
  - `max_yaw_rate` (rad/s clamp, default 0.6)

## Current limitations & TODOs

- Camera calibration / intrinsics stability:
  - The PnP node depends on accurate `/camera_info`. If the saved calibration is off at target resolution the computed tag pose will be noisy.
  - TODO: re-run calibration at the working resolution (1280×720) and verify `camera_info_url` points to the calibrated YAML.

- Pose stability and smoothing:
  - Current PnP code publishes raw solvePnP poses. Consider adding temporal filtering (e.g. simple exponential smoothing or a small Kalman filter) to stabilize TF for control use.

- Lock-on behavior is basic:
  - `hover_yaw_search` implements only a simple yaw P-loop. It does not actively command x/y/z position to hold the tag at a specific range/pose.
  - TODO: implement distance/center control (small x/z linear velocity commands) for better lock and then hand off to a dedicated position controller.

- Safety and flight readiness:
  - The controller publishes velocity setpoints directly to the FCU; ensure all safety checks are performed before enabling control (arm state, GPS / mode checks, geofence, or props removed during bench testing).

- Integration & packaging:
  - Currently several scripts live in `~/` and are run as standalone Python scripts. Consider packaging `apriltag_pnp_broadcaster`, `tag_overlay`, and the TF broadcaster into ROS 2 packages with proper entry points so they can be launched via `ros2 launch` and `ros2 run` consistently.

## Next steps (recommended)

- Recalibrate the camera at the target resolution and verify `/camera_info` values.
- Add a small smoothing filter to the PnP pose before publishing TF.
- Implement a more complete lock behavior that commands small forward/back setpoints based on tag range (z) and lateral corrections.
- Package the home scripts into ROS 2 nodes (optional but recommended) and add launch files that start the full stack (camera, apriltag_ros, pnp broadcaster, overlay, mavros, hover controller).

---

If you want, I can now:
- Create a sample launch file that runs the camera → apriltag_ros → apriltag_pnp_broadcaster → tag_overlay together, or
- Add a simple exponential smoother wrapper around the PnP poses and update the broadcaster to publish the smoothed TF.

Which would you like next?

## Shutdown & Next Session checklist

Notes recorded here so you can power off the Raspberry Pi and resume work quickly next session.

- Scripts and packages (paths):
  - `~/apriltag_pnp_broadcaster.py` — PnP TF broadcaster (uses `/camera_info` + `/detections` → publishes TF `tag36h11:<id>` with parent `camera`).
  - `~/apriltag_tf_broadcaster.py` — alternative detector-relay TF broadcaster.
  - `~/tag_overlay.py` — visualization overlay publishing `/image_with_tags`.
  - `~/ros2_ws/src/tag_hover_controller` — `hover_yaw_search` controller package (SEARCH → LOCK pipeline).

- Quick resume steps (recommended, run in this order after Pi boots and you SSH in):

```bash
# Source ROS 2 (Jazzy) and your workspace
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 1) Launch camera (adjust device and camera_info_url as needed)
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -p video_device:=/dev/video4 \
             -p "image_size:='[1280,720]'" \
             -p time_per_frame.num:=1 -p time_per_frame.den:=30 \
             -p pixel_format:=YUYV \
             -p output_encoding:=rgb8 \
             -p frame_id:=camera \
             -p camera_info_url:=file:///home/mars/.ros/camera_info/camera_1280x720.yaml

# 2) Start AprilTag detector
ros2 run apriltag_ros apriltag_node \
  --ros-args -p image_rect:=/image_raw -p camera_info:=/camera_info \
             --params-file /home/mars/apriltag_params.yaml

# 3) Start the PnP TF broadcaster
python3 ~/apriltag_pnp_broadcaster.py

# 4) (Optional) start overlay and view
python3 ~/tag_overlay.py
ros2 run image_tools showimage --ros-args -r image:=/image_with_tags

# 5) Start MAVROS (example)
ros2 launch mavros apm.launch.py fcu_url:=serial:///dev/ttyAMA0:57600

# 6) Start hover controller (SEARCH mode by default)
ros2 run tag_hover_controller hover_yaw_search --ros-args -p mode:=SEARCH -p rate_hz:=20.0
```

- Verification checklist (after starting stack):
  - Topics to confirm present: `/image_raw`, `/camera_info`, `/detections`, `/tf`, `/mavros/state`, `/mavros/setpoint_velocity/cmd_vel_unstamped`.
  - TF frames: parent `camera`; child frames `tag36h11:<id>` (e.g. `tag36h11:0`).
  - Useful quick checks:

```bash
ros2 topic list
ros2 topic echo /detections
ros2 topic echo /mavros/state
ros2 topic echo /tf
```

- Outstanding TODOs to carry forward (high priority):
  - Recalibrate the camera at the working resolution (1280×720) and update `/home/mars/.ros/camera_info/camera_1280x720.yaml`.
  - Add temporal smoothing to the PnP poses (simple exponential smoother or small Kalman filter) in `~/apriltag_pnp_broadcaster.py` to stabilize TF used by the controller.
  - Implement LOCK-mode improvements in `tag_hover_controller`: yaw P-controller is scaffolded — complete it and add small x/z linear velocity commands (range/centering) for robust lock.
  - Package the home scripts (`apriltag_pnp_broadcaster.py`, `tag_overlay.py`, `apriltag_tf_broadcaster.py`) into ROS 2 packages with entry points and create launch files to run the full stack reliably.

- Safety reminders:
  - Do not enable flight motors or allow props to spin while testing commands that publish to `/mavros/setpoint_velocity/cmd_vel_unstamped` unless the vehicle is safely restrained or props removed.
  - Verify `/mavros/state` shows a connected FCU and a safe mode before publishing control commands.

- Where to find the code (quick reference):
  - `~/apriltag_pnp_broadcaster.py` — PnP TF broadcaster (standalone Python script).
  - `~/apriltag_tf_broadcaster.py` — alternative TF relay.
  - `~/tag_overlay.py` — image overlay for debugging.
  - `~/ros2_ws/src/tag_hover_controller` — hover controller package and node `hover_yaw_search`.

- If you want me to prepare patches while the Pi is offline:
  - Tell me which TODO you want prioritized (e.g., add smoothing, implement LOCK z/x control, or create a launch file). I will prepare a patch that you can apply next session.

-- end of shutdown notes