#!/bin/bash

# Function to handle cleanup on exit
cleanup() {
    echo "Cleaning up..."
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    exit 0
}

# Trap Ctrl+C and call cleanup
trap cleanup INT

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <bag_file> <robot_name>"
  exit 1
fi

# Assign arguments to variables
BAG_FILE=$1
ROBOT_NAME=$2

echo "Starting thermal processing pipeline with:"
echo "Robot name: $ROBOT_NAME"
echo "Bag file: $BAG_FILE"

# Check if bag file exists
if [ ! -f "$BAG_FILE" ]; then
    echo "Error: Bag file '$BAG_FILE' does not exist!"
    exit 1
fi

# Source the ROS2 workspace
WORKSPACE_PATH="/home/dtc/airlab_ws/varunkasi_ws"
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "Error: Workspace path '$WORKSPACE_PATH' does not exist!"
    echo "Please update the script with the correct workspace path."
    exit 1
fi

source "$WORKSPACE_PATH/install/setup.bash"

# Set the environment variable for the robot name
export ROBOT_NAME=$ROBOT_NAME

# Set ROS_DOMAIN_ID to 100 in all tmux panes
export ROS_DOMAIN_ID=100

# Kill any existing session with the same name
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start a new tmux session
SESSION_NAME="thermal_bag_play"
tmux new-session -d -s $SESSION_NAME

# Window 1: Play the bag file
tmux rename-window -t $SESSION_NAME:0 "Bag Playback"
PLAY_CMD="source $WORKSPACE_PATH/install/setup.bash && export ROBOT_NAME=$ROBOT_NAME && export ROS_DOMAIN_ID=100 && ros2 bag play $BAG_FILE"
tmux send-keys -t $SESSION_NAME:0 "$PLAY_CMD" C-m

# We'll wait a moment for the bag to start
sleep 2

# Window 2: Run the mono16_converter
tmux new-window -t $SESSION_NAME -n "Mono16 Converter"
CONVERTER_CMD="source $WORKSPACE_PATH/install/setup.bash && export ROBOT_NAME=$ROBOT_NAME && export ROS_DOMAIN_ID=100 && ros2 run mono16_converter mono16_converter --ros-args -r input_topic:=/${ROBOT_NAME}/hand/sensor/thermal/image_raw -r output_topic:=/${ROBOT_NAME}/hand/sensor/thermal/image_raw/mono8"
tmux send-keys -t $SESSION_NAME:1 "$CONVERTER_CMD" C-m

# Window 3: Run the thermal_calibrator
tmux new-window -t $SESSION_NAME -n "Thermal Calibrator"
CALIBRATOR_CMD="source $WORKSPACE_PATH/install/setup.bash && export ROBOT_NAME=$ROBOT_NAME && export ROS_DOMAIN_ID=100 && ros2 run thermal_radiometry thermal_calibrator --ros-args -r input_topic:=/${ROBOT_NAME}/hand/sensor/thermal/image_raw -r output_topic:=/${ROBOT_NAME}/hand/sensor/thermal/image_calibrated"
tmux send-keys -t $SESSION_NAME:2 "$CALIBRATOR_CMD" C-m

# Wait a moment for the nodes to initialize
sleep 1

# Window 4: Launch the visualizer with the perspective file
tmux new-window -t $SESSION_NAME -n "Visualizer"
VISUALIZER_CMD="source $WORKSPACE_PATH/install/setup.bash && export ROBOT_NAME=$ROBOT_NAME && export ROS_DOMAIN_ID=100 && rqt --standalone thermal_radiometric_visualizer --args $BAG_FILE"
tmux send-keys -t $SESSION_NAME:3 "$VISUALIZER_CMD" C-m

# Window 5: Debugging pane with some useful commands
tmux new-window -t $SESSION_NAME -n "Debugging"
DEBUG_CMD="source $WORKSPACE_PATH/install/setup.bash && export ROBOT_NAME=$ROBOT_NAME && export ROS_DOMAIN_ID=100"
tmux send-keys -t $SESSION_NAME:4 "$DEBUG_CMD" C-m
tmux send-keys -t $SESSION_NAME:4 "echo 'Useful commands:'" C-m
tmux send-keys -t $SESSION_NAME:4 "echo 'timeout 5 ros2 topic list -t | grep thermal  # List thermal topics with types'" C-m
tmux send-keys -t $SESSION_NAME:4 "echo 'timeout 5 ros2 topic echo /${ROBOT_NAME}/hand/sensor/thermal/image_calibrated --no-arr  # Echo calibrated topic headers'" C-m
tmux send-keys -t $SESSION_NAME:4 "echo 'timeout 5