#!/bin/bash

# ========================================================================= #
# THERMAL BAG PROCESSOR AND VISUALIZER                                      #
# ========================================================================= #
# This script processes and visualizes thermal bag files.                   #
# It runs in two distinct phases:                                           #
#  1. PROCESSING: Processes raw thermal data to create calibrated and       #
#     mono8 images, with 1:1 correspondence between all messages            #
#  2. VISUALIZATION: Launches visualizer to explore processed data          #
# ========================================================================= #

# Function to handle cleanup on exit
cleanup() {
    echo "Cleaning up..."
    kill $(jobs -p) 2>/dev/null
    
    # Clean up temporary directory if it exists
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

# Function to print error and exit
print_error() {
    echo "ERROR: $1"
    cleanup
    exit 1
}

# Function to check if a command exists
check_command() {
    if ! command -v $1 &> /dev/null; then
        print_error "Required command '$1' not found. Please install it."
    fi
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
    print_error "Bag file '$INPUT_BAG_FILE' does not exist!"
fi

# Check for required commands
check_command "ros2"

# Source the ROS2 workspace
WORKSPACE_PATH="$(pwd)"
if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
elif [ -f "../install/setup.bash" ]; then
    source "../install/setup.bash"
else
    print_error "Could not find workspace setup.bash. Make sure you're running this from the workspace directory."
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
        print_error "Processed bag directory '$PROCESSED_BAG_DIR' not found. Please run without --skip-processing first."
    fi
    
    print_header "SKIPPING PROCESSING - PROCEEDING TO VISUALIZATION"
else
    print_header "STAGE 1: PROCESSING THERMAL DATA"
    
    # Check if the processed bag already exists
    if [ -d "$PROCESSED_BAG_DIR" ]; then
        read -p "Processed bag already exists. Do you want to overwrite? (y/n): " OVERWRITE
        if [ "$OVERWRITE" != "y" ]; then
            print_progress "Using existing processed bag."
            SKIP_PROCESSING=true
        else
            print_progress "Removing existing processed bag..."
            rm -rf "$PROCESSED_BAG_DIR"
        fi
    fi
    
    if [ "$SKIP_PROCESSING" = false ]; then
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
        
        # Get bag info to estimate processing time
        BAG_INFO=$(ros2 bag info "$INPUT_BAG_FILE")
        DURATION=$(echo "$BAG_INFO" | grep "Duration" | awk '{print $2}')
        MESSAGE_COUNT=$(echo "$BAG_INFO" | grep -A 2 "$RAW_TOPIC" | grep "Count:" | awk '{print $2}')
        
        print_progress "Processing bag with approximately $MESSAGE_COUNT messages over $DURATION..."
        print_progress "This may take some time. Please be patient."
        
        # Play the original bag file slowly to ensure proper processing
        # The --rate option slows down playback to ensure all messages are processed
        print_progress "Playing the original bag file at reduced speed for accurate processing..."
        ros2 bag play --loop-range 1 --rate 0.5 "$INPUT_BAG_FILE"
        
        # Wait for playback to complete
        print_progress "Waiting for all messages to be processed..."
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
            print_error "Processed bag directory not found at $TEMP_DIR/processed_bag"
        fi
        
        print_header "PROCESSING COMPLETE"
    fi
fi

# Verify that all required topics exist in the processed bag
print_header "STAGE 2: LAUNCHING VISUALIZER"
print_progress "Verifying bag contents..."
TOPICS=$(ros2 bag info "$PROCESSED_BAG_DIR" | grep -E "Topic:|Count:" -A 1)

# Check each required topic
MISSING_TOPICS=0
for TOPIC in "$RAW_TOPIC" "$CALIBRATED_TOPIC" "$MONO8_TOPIC"; do
    if ! echo "$TOPICS" | grep -q "$TOPIC"; then
        echo "WARNING: Topic '$TOPIC' not found in the bag."
        MISSING_TOPICS=$((MISSING_TOPICS + 1))
    else
        COUNT=$(echo "$TOPICS" | grep -A 1 "$TOPIC" | grep "Count:" | awk '{print $2}')
        print_progress "Found topic $TOPIC with $COUNT messages"
    fi
done

if [ $MISSING_TOPICS -gt 0 ]; then
    echo "WARNING: Some required topics are missing. Visualization may not work correctly."
    read -p "Continue anyway? (y/n): " CONTINUE
    if [ "$CONTINUE" != "y" ]; then
        print_error "Aborting due to missing topics."
    fi
fi

# Start the visualizer with the processed bag
print_progress "Launching thermal_radiometric_visualizer..."
print_progress "Please wait while the visualizer initializes."

# Launch the visualizer with the processed bag path
rqt --standalone thermal_radiometric_visualizer --args "$PROCESSED_BAG_DIR" &
VISUALIZER_PID=$!

# Give the visualizer a moment to initialize
sleep 3

print_progress "Starting playback of the processed bag..."
print_progress "Wait for all messages to be loaded before interacting with the visualizer."
print_progress "You can tell when loading is complete when the progress bar shows 100%."
print_progress "-----------------------------------------------------------------------------"
print_progress "INSTRUCTIONS FOR USING THE VISUALIZER:"
print_progress "1. Wait for the bag playback to complete (progress bar = 100%)"
print_progress "2. Click the 'Pause' button to stop playback"
print_progress "3. Use '<<' and '>>' buttons to navigate to desired frame"
print_progress "4. Make sure 'Radiometric Mode' is checked"
print_progress "5. Click on any pixel to see the temperature at that location"
print_progress "-----------------------------------------------------------------------------"
print_progress "Press Ctrl+C in this terminal to stop playback and exit."

# Play the processed bag at a slow rate to ensure proper synchronization
ros2 bag play --loop-range 1 --rate 0.3 "$PROCESSED_BAG_DIR"

print_progress "Bag playback complete. The visualizer is now loaded with all messages."
print_progress "You can now use the visualizer controls to navigate frames and view temperatures."
print_progress "Press Ctrl+C in this terminal when you're done to exit."

# Wait for visualizer to exit or user to press Ctrl+C
wait $VISUALIZER_PID

# Clean up
cleanup