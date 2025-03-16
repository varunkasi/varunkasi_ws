#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <bag_file> <robot_name>"
  exit 1
fi

# Assign arguments to variables
BAG_FILE=$1
ROBOT_NAME=$2

# Source the ROS2 workspace
WORKSPACE_PATH="/home/dtc/airlab_ws/varunkasi_ws"
source "$WORKSPACE_PATH/install/setup.bash"

# Set the environment variable for the robot name
export ROBOT_NAME=$ROBOT_NAME

# Set ROS_DOMAIN_ID to 100 in all tmux panes
export ROS_DOMAIN_ID=100

# Start a new tmux session
SESSION_NAME="thermal_bag_play"
tmux new-session -d -s $SESSION_NAME

# Window 1: Play the bag file
tmux rename-window -t $SESSION_NAME:0 "Bag Playback"
tmux send-keys -t $SESSION_NAME:0 "source $WORKSPACE_PATH/install/setup.bash && ros2 bag play $BAG_FILE" C-m

# Window 2: Launch the visualizer with the perspective file
tmux new-window -t $SESSION_NAME -n "Visualizer"
# Use --standalone to launch the thermal_radiometric_visualizer plugin with the bag file path
VISUALIZER_CMD="source $WORKSPACE_PATH/install/setup.bash && rqt --standalone thermal_radiometric_visualizer --args $BAG_FILE"
tmux send-keys -t $SESSION_NAME:1 "$VISUALIZER_CMD" C-m

# Window 3: Debugging pane
DEBUG_CMD="source $WORKSPACE_PATH/install/setup.bash && export ROS_DOMAIN_ID=100 && bash"
tmux new-window -t $SESSION_NAME -n "Debugging"
tmux send-keys -t $SESSION_NAME:2 "$DEBUG_CMD" C-m

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME

# Add a message to indicate the script is running
echo "Tmux session '$SESSION_NAME' started with bag playback, visualizer, and debugging pane."