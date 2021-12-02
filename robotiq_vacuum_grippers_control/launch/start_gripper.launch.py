from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotiq_vacuum_grippers_control',
            executable='RobotiqVacuumGrippersRtuNode.py',
            name='RobotiqVacuumGrippersRtuNode'
        ),
        Node(
            package='robotiq_vacuum_grippers_control',
            executable='robotiq_vacuum_grippers_ctrl.py',
            name='robotiq_vacuum_grippers_ctrl'
        )
    ])