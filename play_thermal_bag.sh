#!/bin/bash

# ========================================================================= #
# THERMAL BAG PROCESSOR AND VISUALIZER                                      #
# ========================================================================= #
# This script processes and visualizes thermal bag files:                   #
#  1. PROCESSING: Extracts raw thermal data and pre-processes it into:      #
#     - Raw thermal data arrays                                             #
#     - Calibrated temperature arrays                                       #
#     - 8-bit visualization arrays                                          #
#  2. VISUALIZATION: Launches the standalone visualizer for exploring data  #
# ========================================================================= #

# Function to handle cleanup on exit
cleanup() {
    echo "Cleaning up..."
    kill $(jobs -p) 2>/dev/null
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
PROCESSED_DATA_DIR="${BASE_NAME}_processed"

print_header "THERMAL PROCESSING AND VISUALIZATION"
echo "Robot name: $ROBOT_NAME"
echo "Input bag file: $INPUT_BAG_FILE"
echo "Skip processing: $SKIP_PROCESSING"

# Check if bag file exists
if [ ! -f "$INPUT_BAG_FILE" ]; then
    print_error "Bag file '$INPUT_BAG_FILE' does not exist!"
fi

# Check for required commands
check_command "python3"

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

# Skip processing if requested
if [ "$SKIP_PROCESSING" = true ]; then
    # Check if processed data exists
    if [ ! -d "$PROCESSED_DATA_DIR" ]; then
        print_error "Processed data directory '$PROCESSED_DATA_DIR' not found. Please run without --skip-processing first."
    fi
    
    print_header "SKIPPING PROCESSING - PROCEEDING TO VISUALIZATION"
else
    print_header "STAGE 1: PROCESSING THERMAL DATA"
    
    # Check if the processed data already exists
    if [ -d "$PROCESSED_DATA_DIR" ]; then
        read -p "Processed data already exists. Do you want to overwrite? (y/n): " OVERWRITE
        if [ "$OVERWRITE" != "y" ]; then
            print_progress "Using existing processed data."
            SKIP_PROCESSING=true
        else
            print_progress "Removing existing processed data..."
            rm -rf "$PROCESSED_DATA_DIR"
        fi
    fi
    
    if [ "$SKIP_PROCESSING" = false ]; then
        print_progress "Starting thermal bag processing..."
        
        # Run the Python processor
        python3 $(rospack find thermal_radiometry)/thermal_radiometry/process_thermal_bag.py \
            "$INPUT_BAG_FILE" "$ROBOT_NAME" --output "$PROCESSED_DATA_DIR"
        
        PROCESSING_RESULT=$?
        
        if [ $PROCESSING_RESULT -ne 0 ]; then
            print_error "Processing failed. Check the error messages above."
        fi
        
        # Check if processing was successful
        if [ ! -d "$PROCESSED_DATA_DIR" ] || [ ! -f "$PROCESSED_DATA_DIR/metadata.json" ]; then
            print_error "Processing failed. Output directory or metadata not found."
        fi
        
        print_header "PROCESSING COMPLETE"
        print_progress "Processed data saved to: $PROCESSED_DATA_DIR"
    fi
fi

# Launch the visualizer
print_header "STAGE 2: LAUNCHING VISUALIZER"
print_progress "Starting thermal visualizer..."

# Launch the standalone Python visualizer
python3 $(rospack find thermal_radiometric_visualizer)/thermal_radiometric_visualizer/offline_visualizer.py \
    --data_dir "$PROCESSED_DATA_DIR"

print_progress "Visualizer closed. Exiting..."
cleanup 