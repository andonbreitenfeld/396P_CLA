#!/bin/bash
# TurtleBot3 SLAM demo using tmux (Gazebo + Cartographer + Teleop + Map Saver)

SESSION="slam_session"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# Start a new tmux session detached
tmux new-session -d -s $SESSION

# --- Window 0: Guide + Quit Control ---
tmux rename-window -t $SESSION:0 'guide'
tmux send-keys -t $SESSION:0 "bash -c '
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
clear

SESSION_NAME=$SESSION

cat <<EOF
=====================================================================
                 TurtleBot3 SLAM (Cartographer) Demo Guide
=====================================================================

This tmux session has 5 tabs (windows):
  [0] guide      - Instructions + quit control
  [1] gazebo     - Gazebo simulation environment
  [2] slam       - Cartographer SLAM + RViz 
  [3] teleop     - Keyboard control (active tab to drive)
  [4] map_saver  - Press Enter anytime to save the map

---------------------------------------------------------------
Navigation Controls:
  Alt + 0 -> guide (current)
  Alt + 1 -> gazebo
  Alt + 2 -> slam
  Alt + 3 -> teleop
  Alt + 4 -> map_saver
---------------------------------------------------------------

To quit the entire SLAM demo:
  Type q and press [Enter]

Tip: Teleop only reads keys when its tab is active.
=====================================================================
EOF

while true; do
  read -p \"Enter q to quit: \" key
  if [[ \$key == \"q\" ]]; then
    echo \"Shutting down all tmux windows...\"
    tmux kill-session -t \$SESSION_NAME
    exit 0
  fi
done
'" C-m

# --- Window 1: Gazebo Simulation ---
tmux new-window -t $SESSION:1 -n 'gazebo'
tmux send-keys -t $SESSION:1 "bash -c '
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
clear
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
'" C-m

# --- Window 2: SLAM (Cartographer + RViz) ---
tmux new-window -t $SESSION:2 -n 'slam'
tmux send-keys -t $SESSION:2 "bash -c '
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
clear
echo \"Waiting for /odom topic...\"
until ros2 topic list | grep -q \"/odom\"; do sleep 1; done
echo \"Launching Cartographer SLAM...\"
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
'" C-m

# --- Window 3: Teleop (keyboard drive) ---
tmux new-window -t $SESSION:3 -n 'teleop'
tmux send-keys -t $SESSION:3 "bash -c '
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
clear
ros2 run turtlebot3_teleop teleop_keyboard
'" C-m

# --- Window 4: Map Saver (Enter to save only) ---
tmux new-window -t $SESSION:4 -n 'map_saver'
tmux send-keys -t $SESSION:4 "bash -c '
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
mkdir -p /root/data
clear
cat <<EOF
=====================================================================
                        Map Saver Terminal
=====================================================================
Press [Enter] anytime to save the current map:
  -> Saves to: /root/data/my_map.yaml (and .pgm)

Return to the Guide tab (Alt + 0) to quit the demo.
=====================================================================
EOF

while true; do
  read -p \"Press [Enter] to save map: \" _
  echo \"Saving map to /root/data/my_map...\"
  ros2 run nav2_map_server map_saver_cli -f /root/data/my_map
  echo \"Map saved.\"
done
'" C-m

# Attach user to the guide tab
tmux select-window -t $SESSION:0
tmux attach -t $SESSION
