# CLA — ME 396P Nav2 Demo

This demo shows how to run the Navigation2 stack in simulation with TurtleBot3, using Docker.  
The workflow has **two stages**:

## Two Stages
1. **SLAM (Cartographer)** — Build a map while localizing simultaneously.  
2. **Localization + Navigation (AMCL + Nav2)** — Use the saved map to localize and navigate.

---

## Docker Setup

### Prereqs
- Docker + Docker Compose installed
- Ubuntu host with X11 (default on most desktops)
- 4 terminals open

### GUI Permission (run every session on the host)
```
xhost +local:root
```

## Build Docker Image
```
docker compose up --build -d
```

You’ll need 4 separate terminals:
- Terminal 1: Gazebo simulation
- Terminal 2: RViz + Cartographer
- Terminal 3: Teleop keyboard control
- Terminal 4: Save map

## Stage 1 - SLAM        `
## Gazebo
```
docker compose exec nav2 bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## RViz + SLAM
```
docker compose exec nav2 bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

## Teleop
- Navigate through simulated environment using keyboard to fill the occupancy map in RViz

```
docker compose exec nav2 bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## Map Saving
- Once occupancy map is full, save it

```
docker compose exec nav2 bash
ros2 run nav2_map_server map_saver_cli -f /root/data/my_map
```

## Stage 2 - Navigation
- Stop Gazebo, RViz, and Teleop (Ctrl+C in each terminal)
- Relaunch Gazebo as before

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

- Start Nav2 with saved map:

```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$(pwd)/data/my_map.yaml
```

## In Rviz
If the map does not appear:
- Change the /map topic from volatile to transient local.

- Use 2D Pose Estimate to set robot’s initial pose
- Use Nav2 Goal to send the robot to a destination
