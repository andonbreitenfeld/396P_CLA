#!/bin/bash
# Clean Navigation2 demo with guide tab and minimal console noise

SESSION="nav_session"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# Start a new tmux session detached
tmux new-session -d -s $SESSION

# --- Window 0: Guide + Quit Control ---
tmux rename-window -t $SESSION:0 'guide'
tmux send-keys -t $SESSION:0 "bash -c '
source /opt/ros/humble/setup.bash
clear

SESSION_NAME=$SESSION

cat <<EOF
=====================================================================
                TurtleBot3 Navigation2 Demo Guide
=====================================================================

This tmux session has 3 tabs (windows):
  [0] guide     - Instructions + quit control
  [1] gazebo    - Gazebo simulation environment
  [2] nav2      - Navigation2 stack (AMCL + RViz)

---------------------------------------------------------------
Navigation Controls:
  Alt + 0 -> guide (current)
  Alt + 1 -> gazebo
  Alt + 2 -> nav2
---------------------------------------------------------------

To quit the entire Navigation2 demo:
  Type q and press [Enter]

=====================================================================
EOF

# Main input loop
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
clear
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
'" C-m

# --- Window 2: Nav2 Stack ---
tmux new-window -t $SESSION:2 -n 'nav2'
tmux send-keys -t $SESSION:2 "bash -c '
source /opt/ros/humble/setup.bash
clear
echo \"Waiting for /odom topic...\"
until ros2 topic list | grep -q \"/odom\"; do sleep 1; done
echo \"Launching Nav2 with saved map...\"
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/root/data/my_map.yaml
'" C-m

# Attach user to the guide tab
tmux select-window -t $SESSION:0
tmux attach -t $SESSION
