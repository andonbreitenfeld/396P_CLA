#!/bin/bash
# TurtleBot3 Navigation2 Survey Demo

SESSION="survey_session"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# Start new tmux session detached
tmux new-session -d -s $SESSION

# --- Window 0: Guide + Quit Control ---
tmux rename-window -t $SESSION:0 'guide'
tmux send-keys -t $SESSION:0 "bash -c '
source /opt/ros/humble/setup.bash
clear

SESSION_NAME=$SESSION

cat <<EOF
=====================================================================
              TurtleBot3 Navigation2 Survey Demo Guide
=====================================================================

This tmux session has 4 tabs (windows):
  [0] guide     - Instructions + quit control (q)
  [1] gazebo    - Gazebo launch
  [2] nav2      - Nav2 + RVIz launch
  [3] survey    - Run survey script

---------------------------------------------------------------
Navigation Controls:
  Alt + 0 -> guide (current)
  Alt + 1 -> gazebo
  Alt + 2 -> nav2
  Alt + 3 -> survey
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
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
'" C-m

# --- Window 2: Nav2 Stack ---
tmux new-window -t $SESSION:2 -n 'nav2'
tmux send-keys -t $SESSION:2 "bash -c '
source /opt/ros/humble/setup.bash
clear
echo \"Waiting for /odom topic...\"
until ros2 topic list | grep -q \"/odom\"; do sleep 0.5; done
echo \"Launching Nav2 with house map...\"
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=./maps/house_map.yaml
'" C-m

# --- Window 3: Python Nav2 Test Script ---
tmux new-window -t $SESSION:3 -n 'survey'
tmux send-keys -t $SESSION:3 "bash -c '
source /opt/ros/humble/setup.bash
clear
echo \"Waiting for /odom topic...\"
until ros2 topic list | grep -q \"/odom\"; do sleep 0.5; done
echo \"Waiting briefly before running nav2_test.py...\"
sleep 3
python3 ./nav2_test.py
'" C-m

# Attach user to the guide tab
tmux select-window -t $SESSION:0
tmux attach -t $SESSION
