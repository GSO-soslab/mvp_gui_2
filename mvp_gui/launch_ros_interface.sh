#!/bin/bash

# This script is designed to be called by the MVP GUI to launch the ROS interface node
# in a new terminal with the correct environment setup.

# Exit immediately if a command exits with a non-zero status.
set -e

echo "--- ROS Interface Launcher Script ---"

# --- Environment Setup ---
echo "Setting up ROS environment..."
# Check if the base ROS setup file exists before sourcing
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "WARNING: /opt/ros/jazzy/setup.bash not found."
fi

# Check if the workspace setup file exists before sourcing
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
else
    echo "WARNING: ~/ros2_ws/install/setup.bash not found."
fi

# Argument 1: Path to the venv activate script (still useful for other env vars)
VENV_ACTIVATE_PATH=$1
echo "Sourcing virtual environment config from: $VENV_ACTIVATE_PATH"
if [ -f "$VENV_ACTIVATE_PATH" ]; then
    source "$VENV_ACTIVATE_PATH"
else
    echo "ERROR: Virtual environment activation script not found. Aborting."
    exit 1
fi

# Argument 2: Path for the PID file
ROS_PID_FILE=$2
echo "PID file will be created at: $ROS_PID_FILE"

# Argument 3: The absolute path to the Python executable in the venv
PYTHON_EXEC=$3
echo "Using Python executable: $PYTHON_EXEC"
if [ ! -x "$PYTHON_EXEC" ]; then
    echo "ERROR: Python executable not found or not executable at $PYTHON_EXEC. Aborting."
    exit 1
fi


# --- Launch Python Script ---
# Use the explicit path to the Python executable to guarantee the correct environment.
echo "Launching ROS interface node..."
"$PYTHON_EXEC" -u -m mvp_gui.ros_interface &
GUI_PID=$!
echo "ROS GUI process started with PID: $GUI_PID"

# Write the PID to the specified file so the main app can manage it
echo $GUI_PID > "$ROS_PID_FILE"

# --- Wait and Cleanup ---
# Wait for the background process to finish
wait $GUI_PID
EXIT_CODE=$?
echo "ROS GUI process (PID: $GUI_PID) exited with code: $EXIT_CODE"

# Keep the terminal open for debugging until the user presses Enter
echo "Press Enter to close this terminal..."
read