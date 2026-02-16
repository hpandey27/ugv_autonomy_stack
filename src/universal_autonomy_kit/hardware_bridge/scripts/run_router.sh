#!/bin/bash
# Helper script to run mavlink-router with the Kit's config

# Locate the config file in the install share directory
CONFIG_FILE=$(ros2 pkg prefix hardware_bridge)/share/hardware_bridge/config/mavlink-router.conf

echo "Starting MAVLink Router..."
echo "Config: $CONFIG_FILE"
echo "  - Endpoint 1: Internal MAVROS (UDP 14540)"
echo "  - Endpoint 2: External GCS    (UDP 14550)"

# Check if mavlink-routerd is installed
if ! command -v mavlink-routerd &> /dev/null
then
    echo "ERROR: mavlink-routerd could not be found."
    echo "Please install it: sudo apt install mavlink-router"
    exit 1
fi

mavlink-routerd -c $CONFIG_FILE
