#!/bin/bash

source /home/mowito/ros1_ws/devel/setup.bash
roscd robotiq_vacuum_grippers_control/scripts/
sudo ./host_socat.sh
roslaunch robotiq_vacuum_grippers_control run_gripper_depth_engine.launch

