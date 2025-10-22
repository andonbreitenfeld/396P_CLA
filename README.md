# CLA — ME 396P Nav2 Demo

This demo runs the **TurtleBot3 Navigation2** stack in simulation using **Docker** and **tmux automation scripts**.  
The workflow is designed to run entirely within one container session — you enter the container once, perform SLAM to create a map, then run Navigation2 using that saved map.

---

## Pre-Requisites

### Docker Setup
Install **Docker** and **Docker Compose** on your host system:  
[https://www.docker.com/get-started/](https://www.docker.com/get-started/)

Verify installation:
```bash
docker --version
docker compose version
```

### Environment Requirements
- Host: **Ubuntu** system with X11 display enabled (default on most desktops)
- GPU acceleration is optional; CPU simulation is sufficient for this demo.

---

## Getting Started

### GUI Permission (on host)
Allow Docker containers to display graphical windows (Gazebo and RViz):
```bash
xhost +local:root
```

### Build the Docker Image (on host)
From the project root directory (where your Dockerfile and `docker-compose.yml` are located):
```bash
docker compose up --build -d
```

This builds the container image and installs:
- ROS 2 Humble
- TurtleBot3 Simulation Packages
- Navigation2 (Nav2)
- Cartographer SLAM
- tmux + automation scripts
- Custom `.tmux.conf` for **Alt + number** shortcuts

### Enter the Container
Once the container is running:
```bash
docker exec -it nav2-demo bash
```

You should now see:
```
root@<container_id>:/#
```

All subsequent steps take place **inside this container**.

---

## Stage 1 — SLAM (Cartographer)

### Launch SLAM
Inside the container:
```bash
bash /root/slam_tmux.sh
```
This starts a tmux session that automatically launches:
- Gazebo simulation
- Cartographer SLAM (RViz + mapping)
- Teleop control
- Map saver utility

### Tab Layout (Shown at Bottom of Terminal)

| Tab | Name | Description |
|------|------|-------------|
| [0] | **guide** | Instructions + quit control (`q`) |
| [1] | **gazebo** | Gazebo Launch/Run |
| [2] | **slam** | Cartographer SLAM + RViz Launch/Run|
| [3] | **teleop** | Keyboard control (must be active tab to drive) |
| [4] | **map_saver** | Press **Enter** anytime to save map |

### Navigating Between Tabs

Use the **Alt + number** shortcuts:
- **Alt + 1** → gazebo  
- **Alt + 2** → slam  
- **Alt + 3** → teleop  
- **Alt + 4** → map_saver  
- **Alt + 0** → guide
- \**Clicking the tab name at bottom of terminal also works*

### Operating SLAM

1. Start in **[0] guide** to review instructions.
2. Wait for **Gazebo** and **RViz** to pop-up.
3. Switch to **[3] teleop**.
4. Before driving:
   - Be able to see **[3] teleop** terminal, **RViz**, and **Gazebo**.
   - Align the starting **RViz** map with **Gazebo** to the best of your ability.
   - Suggested Single Monitor Layout:

       ![SLAM Setup Example](Images/SLAM_Setup_Example.png)
4. How to drive:
   - Be selected into **[3] teleop** terminal.
   - Use **W/A/S/D/X** keys to control the robots linear and angular velocities.
   - \**Controlling velocities to navigate can feel weird at first, remember you can always stop with* **S**.
5. As you move the robot:
   - **Gazebo** shows the simulated robot moving in the environment.
   - **RViz** Displays the occupancy grid map updating in real time
      - Walls and obstacles appear as **black lines**.
      - Unexplored regions are **gray**.
      - Free space is **white**.
6. When the occupancy map appears complete:  
   - Example of a completed map:

       ![Occupancy Map Example](Images/Occupancy_Map_Example.png)
   - Switch to **[4] map_saver**.  
   - Press **Enter** to save the map to:  
     ```
     /root/data/my_map.yaml
     ```
7. Once finished, return to **[0] guide** and enter `q` to quit the session.  
   This stops all processes and returns you to the container shell.

---

## Stage 2 — Navigation (Using Saved Map)

After completing SLAM and saving the map, you can launch the Navigation2 stack.

### Launch Navigation2
Inside the same container session:
```bash
bash /root/nav_tmux.sh
```
This starts a tmux session that launches:
- Gazebo simulation (same world)
- Navigation2 stack (AMCL, planner, RViz)

### Window Layout (Shown at Bottom of Terminal)

| Tab | Name | Description |
|------|------|-------------|
| [0] | **guide** | Instructions + quit control (`q`) |
| [1] | **gazebo** | Simulation world |
| [2] | **nav2** | Navigation2 stack (AMCL + Planner + RViz) |

### Navigating Between Tabs

Use the **Alt + number** shortcuts:
- **Alt + 1** → Gazebo  
- **Alt + 2** → Nav2  
- **Alt + 0** → Guide  

### Operating Navigation2

1. Start in **[0] guide** to review instructions.
2. Wait for **Gazebo** and **RViz** to pop-up.  
   - **RViz** should now display the saved map from the SLAM stage.
   - View **RViz** and **Gazebo** side by side.
   - Suggested Single Monitor Layout with **2D Pose Estimate**:

       ![Nav Setup Example](Images/Nav_Setup_Example.png)
3. In **RViz** use the **2D Pose Estimate** tool to set the robot's initial pose:
   - Click the **2D Pose Estimate** tool in the top tool bar.
   - Click and drag on the map to indicate the robot’s approximate position and heading (can use )
   - Use the **2D Pose Estimate** tool to initialize the robot’s pose based on the saved map (can reference **Gazebo** for rough initial pose).  
4. Once localization is stable, use the **Nav2 Goal** tool to send navigation goals:
   - Click the **Nav2 Goal** button in the top toolbar.
   - Click and drag on the desired destination on the map to define the goal position and orientation.
   - The robot will plan a path (displayed as a colored line) and begin moving toward that goal.
   - This can be repeated to send additional goals.
5. You can monitor progress with **Gazebo** and **RViz**
   - **Gazebo** shows the robot "physically" moving around the simulated world.
   - **RVIZ** shows the planned path, current postion, local costmap, and goal status updates in real time.
6. When finished, return to **[0] guide** terminal and enter `q` to quit the session.  
   This stops all processes and returns you to the container shell.
7. To exit the container:
   ```
   exit
   ```

---

## File Overview

| File | Purpose |
|------|----------|
| `Dockerfile` | Builds ROS 2 Humble + TurtleBot3 + Nav2 + Cartographer + tmux |
| `docker-compose.yml` | Defines container runtime and GUI configuration |
| `.tmux.conf` | Defines Alt + number keybindings and status bar layout |
| `slam_tmux.sh` | Automated SLAM (Gazebo + Cartographer + Teleop + Map Saver) |
| `nav_tmux.sh` | Automated Navigation2 (Gazebo + AMCL + Planner + RViz) |

---

## Summary

This workflow keeps all operations within a single container session.  
You build once, enter once, and use tmux automation to switch seamlessly between SLAM and Navigation without managing multiple terminals.  
Mapping, saving, localization, and navigation are all handled through structured tmux windows with simple keyboard shortcuts.