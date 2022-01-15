#!/usr/bin/python3
# Author : safar V
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sudo_password = 'mowito'
    check_socat_connections = 'lsof -i:54321'
    check_socat_connections =check_socat_connections.split()
    
    echo_sudo_cmd = subprocess.Popen(['echo',sudo_password], stdout=subprocess.PIPE)
    check_socat_connections_cmd = subprocess.Popen(['sudo','-S'] + check_socat_connections, stdin=echo_sudo_cmd.stdout, stdout=subprocess.PIPE)
    check_socat_connections_str = check_socat_connections_cmd.stdout.read().decode('utf-8')
    #Todo : remove hardcoded path, get_package_share_directory not working
    if(len(check_socat_connections_str)==0):
        terminal_output = subprocess.call(["sudo","./host_socat.sh"],cwd="/home/mowito/gripper_ws/src/gripper_ros1_driver/scripts/")

    config_gripper_node = os.path.join(
        get_package_share_directory('gripper_ros2'),
        'config',
        'gripper_config.yaml'
        )
    return LaunchDescription([
        Node(
            package='gripper_ros2',
            executable='gripper_ros2_node',
            output='screen',
            parameters = [config_gripper_node]
        )
    ])
