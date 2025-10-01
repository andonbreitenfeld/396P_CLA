# CLAude — ME 396P Nav2 Demo 

## Docker Setup
## Prereqs
- Docker + Docker Compose installed
- Ubuntu host with X11 (default on most desktops)
- 3 Terminals opened

## GUI permission (run every session)
```
xhost +local:root
```

## Build Docker Image
```
docker compose up --build -d
```

## Gazebo
- Run this in terminal 1

Launch Docker Container:
```
docker compose exec nav2 bash
```

Launch Gazebo:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Teleop Control
- Run this in terminal 2

Launch Docker Container:
```
docker compose exec nav2 bash
```

Launch Teleop Control:
```
ros2 run turtlebot3_teleop teleop_keyboard
```

## RViz2 / Map Saving
- Run this in terminal 3

Launch Docker Container:
```
docker compose exec nav2 bash
```

Save Maps:
```
ros2 run nav2_map_server map_saver_cli -f /root/data/my_map
```
