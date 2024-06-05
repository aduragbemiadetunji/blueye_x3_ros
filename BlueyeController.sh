#!/bin/bash

# Source the ROS setup file
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash



# Initialize pyenv
export PATH="$HOME/.pyenv/bin:$PATH"
if command -v pyenv 1>/dev/null 2>&1; then
  eval "$(pyenv init --path)"
  eval "$(pyenv init -)"
fi

# Activate the virtual environment
pyenv activate sdk-env 

sleep 3 


# Run your roslaunch file
roslaunch blueye_x3_ros blueyeSensorData.launch &


sleep 3

roslaunch blueye_x3_ros allModel.launch &

sleep 3

# Run additional ROS nodes or scripts if needed
rosrun blueye_x3_ros GUIcontroller.py

# Wait for all background processes to finish
wait

