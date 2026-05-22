# Patrol AI

This repository contains only the files I wrote and shared for the Patrol AI robot project. I did not add a complete ROS 2 workspace here — I only included the launch scripts, the navigation parameter file, the Python nodes for anomaly detection and dashboard orchestration, and the HTML dashboard itself.

## What this project does

I built a patrol robot pipeline that combines 2D LiDAR mapping, Nav2 navigation, camera-based anomaly detection, and a remote dashboard.

The workflow is:

1. Use `slam_toolbox` to build a 2D map of the environment.
2. Save the map to disk.
3. Use Nav2 to navigate on the saved map.
4. Run the spotter node to validate camera detections against the map.
5. Use the web dashboard to control the robot and monitor alerts.

This project is designed so the robot can navigate normally while the camera system checks for unexpected obstacles and anomalies.

## File overview

### `launch/limo_start.launch.py`
This is the main start-up launch file that I modified.

What it does:
- Loads the robot model from `limo_description/urdf/limo_four_diff.xacro` and publishes it through `robot_state_publisher`.
- Starts `twist_mux`, which merges teleop commands and navigation-generated velocity commands into a single `/cmd_vel` output.
- Includes the `limo_base` bringup launch and the YDLidar launch.
- Optionally starts `slam_toolbox` with `async_slam_toolbox_node` when `use_slam` is enabled.

I use this launch file to start the robot hardware stack and the mapping/navigation control flow.

### `launch/limo_mapping.launch.py`
This launch file is my dedicated SLAM launch.

What it does:
- Starts `slam_toolbox` in async mapper mode.
- Starts `rviz2` so I can inspect the map while mapping.

This is the mapping mode I use before I save the map and switch to autonomous navigation.

### `launch/limo_nav2.launch.py`
This launch file starts Nav2 navigation on a saved map.

What it does:
- Loads a map from the `limo_bringup/maps` folder by default.
- Loads `param/nav2.yaml` as the Nav2 configuration.
- Wraps the Nav2 bringup launch with a remap so `/cmd_vel` becomes `/cmd_vel_nav`.
- Launches `rviz2` with the Nav2 visualization setup.

This file is the main entry point for using a saved map and executing navigation missions.

### `param/nav2.yaml`
This file contains the navigation tuning I created.

What is included:
- `amcl` tuning for a differential drive robot with moderate encoder noise and hallway motion.
- `bt_navigator` configuration using the standard Nav2 behavior tree.
- `controller_server` settings for the DWB local planner, including velocity limits, acceleration, critic weights, and trajectory sampling.
- `local_costmap` configured for `LaserScan` obstacles plus a `VoxelLayer` from the depth camera point cloud.
- `global_costmap` configured with static and obstacle layers.
- Planner and recovery server tuning.
- Lifecycle managers for both localization and navigation.

The key idea here is combining 2D LiDAR with depth-camera voxel obstacle detection so the robot can see things that the laser might miss.

### `scripts/spotter.py`
This is the anomaly detection node I built.

How it works:
- Subscribes to `/map` to keep the latest occupancy grid.
- Subscribes to `/amcl_pose` to know the robot's pose in the map.
- Subscribes to `/camera/depth/image_raw` and `/camera/color/image_raw`.
- Runs YOLOv8 on the color image to detect objects at about 2 FPS.
- Uses depth data and the camera field of view to estimate object distance and lateral offset.
- Projects those detections into the map frame and checks whether the map expects that location to be empty.

If it finds a new anomaly, it:
- publishes a marker on `/spotter/anomaly_markers`
- publishes an alert JSON on `/spotter/alerts`
- saves an annotated image for the dashboard
- sends zero velocity commands on `/cmd_vel_teleop`

This node is my safety/validation layer: it only flags objects that the current map says should not exist.

### `scripts/dashboard_commander.py`
This is the command bridge for the web dashboard.

How it works:
- Subscribes to `/dashboard_cmd`.
- On `START_MAPPING`, it launches the SLAM mapping flow.
- On `START_NAV:<map_name>`, it launches navigation with the requested map.
- On `STOP`, it stops the current launched process.

The dashboard uses this node to start mapping or navigation without manually typing ROS launch commands.

### `web/index.html`
This is the web dashboard UI I created.

What it provides:
- Connects to ROS using `rosbridge` and `roslibjs`.
- Sends teleop velocity commands to `/cmd_vel_teleop`.
- Sends dashboard commands to `/dashboard_cmd`.
- Displays the robot pose from `/odom`.
- Shows a live camera feed from the compressed image topic.
- Can switch between raw camera feed and YOLO-annotated feed.
- Displays anomaly alerts from `/spotter/alerts`.

It is designed to be the operator interface for remote control, mission start/stop, and anomaly monitoring.

## How the project fits together

The whole system is built around two phases:

1. Mapping phase
   - Launch `limo_mapping.launch.py`.
   - Build a 2D map with `slam_toolbox`.
   - Save the map to disk.

2. Navigation phase
   - Launch `limo_nav2.launch.py` with the saved map.
   - Start `spotter.py` to watch for new obstacles.
   - Use the web dashboard for manual override, mission control, and alerts.

While Nav2 handles autonomous path planning and local obstacle avoidance, the spotter node acts as a visual check against the saved map. That gives me a layer of anomaly detection that can identify new objects or unexpected obstacles in the scene.

## Startup commands

These are the exact commands I use to get the full system running:

```bash
ros2 launch limo_bringup limo_start.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 launch orbbec_camera dabai.launch.py enable_color:=true enable_depth:=true enable_point_cloud:=true enable_ir:=false point_cloud_qos:=SENSOR_DATA depth_qos:=SENSOR_DATA
ros2 run tf2_ros static_transform_publisher 0.084 0.0 0.01 0.0 0.0 0.0 base_link camera_link
ros2 run image_transport republish raw compressed --ros-args --remap in:=/camera/color/image_raw --remap out/compressed:=/camera/color/image_raw/compressed
python3 -m http.server 7000
x11vnc -display :0 -forever -nopw -shared -rfbport 5900
websockify --web /usr/share/novnc/ 6080 localhost:5900
python3 dashboard_commander.py
ros2 run limo_bringup spotter.py
```

## Why I kept this repo simple

I kept this repository limited to the files I actually wrote and shared. I did not add a full ROS workspace, package manifests, or build files. This is intentionally the working core of the project, not a reconstructed ROS stack.
