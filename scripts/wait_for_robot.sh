#!/bin/bash

# Wait for ROS services to be available
echo "Waiting for robot services to be available..."
sleep 5

# Load the remote control program
echo "Loading remote_control.urp program..."
rosservice call /ur/ur_hardware_interface/dashboard/load_program "filename: 'remote_control.urp'"
sleep 2

# Play the program
echo "Starting remote control program..."
rosservice call /ur/ur_hardware_interface/dashboard/play "{}"
sleep 2

echo "Robot is ready for remote control"