# Robotiq

## Setup Instructions 

```
- cd $HOME
- mkdir -p grip/src
- cd grip/src
- git clone https://github.com/mowito/gripper_ros1_driver.git -b ur_ros2
- cd ..
- colcon build
```
## Launch instructions

```
- source install/setup.bash
- ros2 launch robotiq_vacuum_grippers_control start_gripper.launch.py
```
