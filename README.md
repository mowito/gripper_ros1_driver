# Robotiq

## Setup Instructions 

```
- cd $HOME
- mkdir -p grip_ws/src
- git clone https://github.com/mowito/gripper_ros1_driver.git -b sv_ros2_total_revamp
- cd ..
- colcon build
```
## Launch instructions

```
- source install/setup.bash
- ros2 launch gripper_ros2 gripper_start.launch.py
```
Creating socat ports is in-built into the Launch file, no need to run `./host_socat.sh` script. 
Need to take care of path to `./host_socat.sh` in the launch file, its currently hard coded.
