# F1Tenth Sim Testing Guide

## Overview

This repo runs a full autonomous racing stack inside a Docker container using the F1Tenth gym simulator (ROS 2 Humble). Visualization is done through Foxglove Studio over WebSocket.

The autonomous pipeline is:

```
Global Planner → Local Planner → Control → /drive (sim)
```

- **Global Planner**: Loads the Spielberg centerline CSV and publishes all waypoints once as a `PoseArray` on `/global_planner`
- **Local Planner**: Subscribes to `/global_planner` and `/ego_racecar/odom`. On each odom tick, finds the best waypoint in a look-ahead annulus (0.5m–1.22m) and publishes it on `/drive_to`
- **Control**: Subscribes to `/drive_to` and `/ego_racecar/odom`. Computes the heading error to the target waypoint and publishes an `AckermannDriveStamped` message on `/drive`
- **Sim Bridge**: F1Tenth gym node that receives `/drive` and steps the physics simulation forward, publishing `/ego_racecar/odom` and `/scan`

---

## Prerequisites

- Docker Desktop installed and running
- Foxglove Studio installed (https://foxglove.dev)
- This repo cloned locally

---

## Step 1 — Start the Container

From the `f1tenth/` directory on your Mac:

```bash
docker compose up -d
```

This builds the image (if needed) and starts the container in the background. The `./ws` folder is mounted into `/root/ws` inside the container. Port `8765` is forwarded to your Mac for Foxglove.

---

## Step 2 — Terminal 1: Launch the Simulator

Open a terminal and exec into the container:

```bash
docker exec -it <container_id> bash
```

Find the container ID with `docker ps` if needed.

Inside the container, source and launch the sim:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

This starts:
- The F1Tenth gym bridge (physics sim)
- The Nav2 map server (Spielberg map)
- The ego robot state publisher
- The Foxglove WebSocket bridge on port 8765

> **Note**: If you changed any code, run `colcon build` before sourcing and launching.

---

## Step 3 — Terminal 2: Launch the Autonomous Stack

Open a second terminal and exec into the same container:

```bash
docker exec -it <container_id> bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch sig_stack stack_launch.py
```

This starts:
- `global_planner` — loads `Spielberg_centerline.csv` and publishes 864 waypoints
- `local_planner` — selects the best look-ahead waypoint on each odom tick
- `control` — converts the target waypoint into a steering angle and speed command

You should see:
```
[global_planner-1] Loaded 864 waypoints from centerline
[global_planner-1] Published global path
```

---

## Step 4 — Connect Foxglove

1. Open Foxglove Studio
2. Click **Open Connection**
3. Select **Rosbridge WebSocket**
4. Enter: `ws://localhost:8765`
5. Click **Open**

Add panels as needed (e.g., 3D view to see the car and map, plot panels for odom/scan).

---

## Step 5 — Verify the Stack is Working

In a third terminal (inside the container), check that all nodes are running:

```bash
ros2 node list
```

Expected nodes:
```
/bridge
/ego_robot_state_publisher
/foxglove_bridge
/lifecycle_manager_localization
/map_server
/global_planner
/local_planner
/control
```

Check that drive commands are being published:

```bash
ros2 topic echo /drive
```

You should see `AckermannDriveStamped` messages with non-zero `speed` and `steering_angle`.

---

## Manual Drive Test (without the stack)

To verify the sim is working independently, publish a single drive command:

```bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}" --once
```

The car should move forward in Foxglove.

---

## Rebuilding After Code Changes

Only rebuild when you change code. From inside the container:

```bash
colcon build
source install/setup.bash
```

To rebuild only the sig_stack package (faster):

```bash
colcon build --packages-select sig_stack
source install/setup.bash
```

---

## Key Files

| File | Purpose |
|------|---------|
| `ws/src/f1tenth_gym_ros/launch/gym_bridge_launch.py` | Sim launch — starts gym bridge, map server, foxglove bridge |
| `ws/src/f1tenth_gym_ros/config/sim.yaml` | Sim config — map path, starting pose, scan params |
| `ws/src/sig_stack/launch/stack_launch.py` | Stack launch — starts global planner, local planner, control |
| `ws/src/sig_stack/sig_stack/global_planner.py` | Loads centerline CSV, publishes PoseArray once |
| `ws/src/sig_stack/sig_stack/localplanner.py` | Look-ahead waypoint selector |
| `ws/src/sig_stack/sig_stack/control.py` | Pure pursuit heading controller |
| `ws/src/sig_stack/maps/Spielberg_centerline.csv` | 864-waypoint centerline for the Spielberg map |
| `docker-compose.yml` | Container config — port 8765 forwarded, `./ws` mounted |
| `Dockerfile` | Image definition — ROS Humble + foxglove-bridge |

---

## Known Limitations

- The speed in `local_planner.py` is a hardcoded placeholder (`1.0 m/s`). Velocity profiling along the race line is not yet implemented.
- The `control.py` `get_new_speed()` function is a pass-through placeholder.
- The local planner iterates through all 864 waypoints on every odom tick — fine for now but will need optimization for real-time use.
