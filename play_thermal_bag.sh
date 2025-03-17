#!/bin/bash

# Function to handle cleanup on exit
cleanup() {
    echo "Cleaning up..."
    kill $(jobs -p) 2>/dev/null
    
    # Clean up temporary directory if it exists and we created one
    if [ -n "$TEMP_DIR" ] && [ -d "$TEMP_DIR" ]; then
        echo "Removing temporary directory: $TEMP_DIR"
        rm -rf "$TEMP_DIR"
    fi
    
    echo "Done."
    exit 0
}

# Function to print section headers
print_header() {
    echo ""
    echo "========================================================================="
    echo "$1"
    echo "========================================================================="
}

# Function to print progress
print_progress() {
    echo "-> $1"
}

# Trap Ctrl+C and call cleanup
trap cleanup INT

# Check if the correct number of arguments is provided
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <bag_file> <robot_name> [--skip-processing]"
  echo "  <bag_file>: Path to the thermal bag file"
  echo "  <robot_name>: Robot name (e.g., spot1, spot2)"
  echo "  --skip-processing: Optional flag to skip processing and go straight to visualization"
  exit 1
fi

# Assign arguments to variables
INPUT_BAG_FILE=$1
ROBOT_NAME=$2
SKIP_PROCESSING=false

# Check for optional skip-processing flag
if [ "$#" -eq 3 ] && [ "$3" == "--skip-processing" ]; then
    SKIP_PROCESSING=true
fi

# Create output paths
BASE_NAME=$(basename "$INPUT_BAG_FILE" .bag)
PROCESSED_BAG_DIR="${BASE_NAME}_processed"

print_header "THERMAL PROCESSING AND VISUALIZATION"
echo "Robot name: $ROBOT_NAME"
echo "Input bag file: $INPUT_BAG_FILE"
echo "Skip processing: $SKIP_PROCESSING"

# Check if bag file exists
if [ ! -f "$INPUT_BAG_FILE" ]; then
    echo "Error: Bag file '$INPUT_BAG_FILE' does not exist!"
    exit 1
fi

# Source the ROS2 workspace
WORKSPACE_PATH="$(pwd)"
if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
elif [ -f "../install/setup.bash" ]; then
    source "../install/setup.bash"
else
    echo "Error: Could not find workspace setup.bash. Make sure you're running this from the workspace directory."
    exit 1
fi

# Set the environment variables
export ROBOT_NAME=$ROBOT_NAME
export ROS_DOMAIN_ID=100

# Define topic names
RAW_TOPIC="/${ROBOT_NAME}/hand/sensor/thermal/image_raw"
CALIBRATED_TOPIC="/${ROBOT_NAME}/hand/sensor/thermal/image_calibrated"
MONO8_TOPIC="/${ROBOT_NAME}/hand/sensor/thermal/image_raw/mono8"

# Print topic information
echo "Raw topic: $RAW_TOPIC"
echo "Calibrated topic: $CALIBRATED_TOPIC"
echo "Mono8 topic: $MONO8_TOPIC"

# Skip processing if requested
if [ "$SKIP_PROCESSING" = true ]; then
    # Check if processed bag exists
    if [ ! -d "$PROCESSED_BAG_DIR" ]; then
        echo "Error: Processed bag directory '$PROCESSED_BAG_DIR' not found."
        echo "Please run without --skip-processing first to create the processed bag."
        exit 1
    fi
    
    print_header "SKIPPING PROCESSING - LAUNCHING VISUALIZER"
else
    print_header "STAGE 1: PROCESSING THERMAL DATA"
    
    # Create temporary directory for intermediate files
    TEMP_DIR=$(mktemp -d)
    print_progress "Created temporary directory: $TEMP_DIR"
    
    # Start processors in background
    print_progress "Starting thermal_calibrator node..."
    ros2 run thermal_radiometry thermal_calibrator \
      --ros-args -r input_topic:=$RAW_TOPIC \
      -r output_topic:=$CALIBRATED_TOPIC &
    CALIBRATOR_PID=$!
    
    print_progress "Starting mono16_converter node..."
    ros2 run mono16_converter mono16_converter \
      --ros-args -r input_topic:=$RAW_TOPIC \
      -r output_topic:=$MONO8_TOPIC &
    CONVERTER_PID=$!
    
    # Give the nodes a moment to initialize
    print_progress "Waiting for nodes to initialize..."
    sleep 3
    
    # Start recording processed topics
    print_progress "Recording processed topics to a new bag file..."
    ros2 bag record -o "$TEMP_DIR/processed_bag" \
      $RAW_TOPIC \
      $CALIBRATED_TOPIC \
      $MONO8_TOPIC &
    RECORD_PID=$!
    
    # Give the recorder a moment to initialize
    sleep 2
    
    # Play the original bag file
    print_progress "Playing the original bag file to process data..."
    print_progress "This may take some time depending on the size of the bag file..."
    ros2 bag play --loop-range 1 "$INPUT_BAG_FILE"
    
    # Wait a moment to ensure all messages are processed
    print_progress "Waiting for all processors to complete..."
    sleep 5
    
    # Kill background processes
    print_progress "Stopping all processors..."
    kill $RECORD_PID $CALIBRATOR_PID $CONVERTER_PID 2>/dev/null
    
    # Wait for all processes to terminate
    wait $RECORD_PID $CALIBRATOR_PID $CONVERTER_PID 2>/dev/null || true
    
    # Move the processed bag to the final destination
    print_progress "Finalizing processed bag file..."
    if [ -d "$TEMP_DIR/processed_bag" ]; then
        # Remove any existing processed bag directory
        if [ -d "$PROCESSED_BAG_DIR" ]; then
            rm -rf "$PROCESSED_BAG_DIR"
        fi
        
        # Move the new processed bag to the final location
        mv "$TEMP_DIR/processed_bag" "$PROCESSED_BAG_DIR"
        print_progress "Created processed bag directory: $PROCESSED_BAG_DIR"
    else
        echo "Error: Processed bag directory not found at $TEMP_DIR/processed_bag"
        cleanup
        exit 1
    fi
    
    print_header "PROCESSING COMPLETE"
fi

# Verify that all required topics exist in the processed bag
print_header "STAGE 2: LAUNCHING VISUALIZER"
print_progress "Verifying bag contents..."
TOPICS=$(ros2 bag info "$PROCESSED_BAG_DIR" | grep -E "Topic:|Count:" -A 1)

# Check each required topic
for TOPIC in "$RAW_TOPIC" "$CALIBRATED_TOPIC" "$MONO8_TOPIC"; do
    if ! echo "$TOPICS" | grep -q "$TOPIC"; then
        echo "Warning: Topic '$TOPIC' not found in the bag."
    else
        COUNT=$(echo "$TOPICS" | grep -A 1 "$TOPIC" | grep "Count:" | awk '{print $2}')
        print_progress "Found topic $TOPIC with $COUNT messages"
    fi
done

# Start the visualizer with the processed bag
print_progress "Launching thermal_radiometric_visualizer..."
rqt --standalone thermal_radiometric_visualizer --args "$PROCESSED_BAG_DIR" &
VISUALIZER_PID=$!

# Give the visualizer a moment to initialize
sleep 2

# Play the processed bag
print_progress "Playing the processed bag file..."
print_progress "Use the visualizer controls to pause, navigate frames, and analyze temperatures."
print_progress "Press Ctrl+C in this terminal to stop playback and exit."
ros2 bag play "$PROCESSED_BAG_DIR"

# Wait for visualizer to exit or user to press Ctrl+C
wait $VISUALIZER_PID

# Clean up
cleanup