#!/bin/bash

# ==============================================================================
#  Robot Circular Motion Launcher Script
# ==============================================================================
# This script provides a user-friendly interface to run a ROS 2 motion node.
# It uses tmux to manage and display the robot driver and the motion script
# in separate, viewable panes.
#
# Features:
#   - Launches the required RM-75 robot driver.
#   - Prompts the user for all motion parameters.
#   - Validates user input against predefined safety and operational limits.
#   - Prevents execution with invalid parameters.
#   - Displays a summary of the parameters before execution.
# ==============================================================================

# --- Configuration ---
# !! IMPORTANT !! -> Update these variables to match your ROS 2 package and node names.
ROS_PKG_NAME="my_ros_pkg"
ROS_NODE_NAME="circular"
TMUX_SESSION="111"

# --- Style and Color Definitions ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ==============================================================================
#  HELPER FUNCTIONS FOR INPUT VALIDATION
# ==============================================================================

# --- Function to validate an integer within a given range ---
# Arguments: $1: Prompt Message, $2: Variable Name to store result, $3: Min value, $4: Max value
function get_validated_integer() {
    local prompt="$1"
    local var_name="$2"
    local min="$3"
    local max="$4"
    local input

    while true; do
        read -p "$(echo -e ${CYAN}"$prompt"${NC})" input
        # Check if input is a valid integer
        if [[ "$input" =~ ^[0-9]+$ ]]; then
            # Check if input is within the allowed range
            if [[ "$input" -ge "$min" && "$input" -le "$max" ]]; then
                eval "$var_name=$input" # Set the global variable
                return 0
            else
                echo -e "${RED}Error: Input must be between $min and $max.${NC}"
            fi
        else
            echo -e "${RED}Error: Invalid input. Please enter a whole number.${NC}"
        fi
    done
}

# --- Function to validate a floating-point number within a given range ---
# Arguments: $1: Prompt Message, $2: Variable Name to store result, $3: Min value, $4: Max value
function get_validated_float() {
    local prompt="$1"
    local var_name="$2"
    local min="$3"
    local max="$4"
    local input

    while true; do
        read -p "$(echo -e ${CYAN}"$prompt"${NC})" input
        # Check if input is a valid floating point number format
        if [[ "$input" =~ ^[0-9]*\.?[0-9]+$ ]]; then
            # Use 'bc' (basic calculator) for floating point comparison
            is_ge_min=$(echo "$input >= $min" | bc -l)
            is_le_max=$(echo "$input <= $max" | bc -l)
            if [[ "$is_ge_min" -eq 1 && "$is_le_max" -eq 1 ]]; then
                eval "$var_name=$input" # Set the global variable
                return 0
            else
                echo -e "${RED}Error: Input must be between $min and $max.${NC}"
            fi
        else
            echo -e "${RED}Error: Invalid input. Please enter a number (e.g., 0.3).${NC}"
        fi
    done
}

# ==============================================================================
#  MAIN SCRIPT LOGIC
# ==============================================================================

# --- 1. Get User Input ---
clear
echo -e "${GREEN}=====================================================${NC}"
echo -e "${GREEN} Interactive Robot Circular Motion Launcher ${NC}"
echo -e "${GREEN}=====================================================${NC}"
echo "Please provide the motion parameters below."
echo ""

# Get validated parameters from the user
get_validated_integer "Enter JOINT speed (1-100): " SPEED_JOINT 1 100
get_validated_integer "Enter LINEAR speed (1-100): " SPEED_LINEAR 1 100
get_validated_integer "Enter CIRCULAR speed (1-100): " SPEED_CIRCULAR 1 100
get_validated_float "Enter circular height in meters (0.25 - 0.40): " CIRCULAR_HEIGHT 0.25 0.40
get_validated_integer "Enter number of loops (> 0): " CIRCULAR_LOOP 1 9999
get_validated_float "Enter circle radius in meters (0.0 - 0.1): " RADIUS 0.0 0.1

# --- 2. Display a Summary of Parameters ---
echo ""
echo -e "${YELLOW}-----------------------------------------------------${NC}"
echo -e "${YELLOW} Parameters Summary:${NC}"
echo -e "${YELLOW}-----------------------------------------------------${NC}"
echo -e "  Speeds (Joint/Linear/Circular): ${GREEN}$SPEED_JOINT% / $SPEED_LINEAR% / $SPEED_CIRCULAR%${NC}"
echo -e "  Circular Height:                ${GREEN}${CIRCULAR_HEIGHT}m${NC}"
echo -e "  Circular Loops:                 ${GREEN}${CIRCULAR_LOOP}${NC}"
echo -e "  Radius:                         ${GREEN}${RADIUS}m${NC}"
echo -e "${YELLOW}-----------------------------------------------------${NC}"
echo ""
read -p "Press [Enter] to start the robot, or [Ctrl+C] to cancel."

# --- 3. Build the ROS 2 Command ---
ROS_COMMAND="ros2 run ${ROS_PKG_NAME} ${ROS_NODE_NAME} --ros-args \
    -p circular_height:=${CIRCULAR_HEIGHT} \
    -p speed_joint:=${SPEED_JOINT} \
    -p speed_linear:=${SPEED_LINEAR} \
    -p speed_circular:=${SPEED_CIRCULAR} \
    -p circular_loop:=${CIRCULAR_LOOP} \
    -p radius:=${RADIUS}"

# --- 4. Launch in Tmux ---
echo -e "\n${GREEN}Starting tmux session: ${TMUX_SESSION}...${NC}"

# Kill any existing session with the same name to ensure a clean start
tmux kill-session -t $TMUX_SESSION 2>/dev/null

# Create a new detached tmux session
tmux new-session -d -s $TMUX_SESSION -n "Driver"

# Send the driver launch command to the first window
tmux send-keys -t $TMUX_SESSION:Driver "echo 'Launching RM-75 Driver...'; ros2 launch rm_driver rm_75_driver.launch.py" C-m

echo "Waiting 2 seconds for the driver to initialize..."
sleep 2

# Create a new window for the motion control script
tmux new-window -t $TMUX_SESSION -n "Motion"

# Send the fully constructed ROS command to the new window
tmux send-keys -t $TMUX_SESSION:Motion "$ROS_COMMAND" C-m

# Split the window vertically for better viewing
tmux split-window -h -t $TMUX_SESSION:Driver

# Attach to the tmux session
tmux attach-session -t $TMUX_SESSION
