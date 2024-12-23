#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <speed_value>"
    exit 1
fi

CONFIG_FILE="/home/nicolaslauzon/ws/uni/hedgehog_ws/src/hedgehog_system/config/teleop.yaml"

NEW_SCALE_VALUE="$1"

# Validate the file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: File '$CONFIG_FILE' not found."
    exit 1
fi

# Use yq to update the scale value
if command -v yq >/dev/null 2>&1; then
    yq -i ".joy_teleop_node.ros__parameters.human_control.axis_mappings.drive-speed.scale = $NEW_SCALE_VALUE" $CONFIG_FILE
    echo "Successfully updated the scale value to $NEW_SCALE_VALUE in $CONFIG_FILE."
else
    echo "Error: 'yq' is not installed. Install yq and try again."
    exit 1
fi

if screen -list | grep -q "bag_recording"; then
    echo "A screen session named 'bag_recording' is already running."
    exit 1
fi

# Run the commands in a new screen session
screen -dmS bag_recording bash -c "source /home/nicolaslauzon/ws/uni/hedgehog_ws/install/setup.bash && ros2 launch hedgehog_system record_mapping.launch.py"
echo "Mapping is being recorded in a screen session named 'bag_recording'."
