#!/bin/bash

echo "Launching PS3 Bluetooth Script"
source ${ROS_ROOT}/tools/rosbash/rosbash
roscd ps3joy 
sudo ./ps3joy.py --inactivity-timeout=180
