# On Display

code for [On Display 2022](https://ondisplay.us/) with Amy Eguchi and Hortense Gerardo at Franklin Antonio Hall.

# Running

`conda activate xarm`

# Setup

With a fresh install, this worked

Follow these instructions: https://github.com/xArm-Developer/xarm_ros2/tree/galactic#3-preparation

## Usage

```
cd ~/dev_ws
source install/setup.bash
```
	
simulating with moveit 
```
ros2 launch xarm_moveit_config xarm7_moveit_fake.launch.py
```

RVIZ to visualize:
```
ros2 launch xarm_planner xarm7_planner_fake.launch.py
```

in another terminal window, python to drive: 
```
cd ~/dev_ws
source install/setup.bash
ros2 launch xarm_planner test_xarm_planner_api_joint.launch.py dof:=7 robot_type:=xarm
```
from xarm planner example: https://github.com/xArm-Developer/xarm_ros2/tree/galactic#57-xarm_planner

### Rviz and control with ROS

launch rviz simualtion:
```
ros2 launch xarm_planner xarm7_planner_fake.launch.py
```

movements (joint or pose-based):
```
ros2 launch xarm_planner test_xarm_planner_client_joint.launch.py dof:=7
ros2 launch xarm_planner test_xarm_planner_client_pose.launch.py dof:=7
```
from: https://github.com/xArm-Developer/xarm_ros2/tree/galactic#57-xarm_planner

### More about Moveit
- https://moveit.ros.org/documentation/applications/
- https://github.com/o2as/ur-o2as/ robot arms with sticks and string, playing with a diabolo. Uses MoveIt and bio_ik. Includes a Gazebo simulation plugin and Rviz visualization.
- bioIK: https://github.com/TAMS-Group/bio_ik
- trak-IK: https://bitbucket.org/traclabs/trac_ik/src/master/trac_ik_python/

# Real-Time Control
opencv camera in put in ROS2 galactic https://automaticaddison.com/getting-started-with-opencv-in-ros-2-galactic-python/

ROS noetic / pybullet https://github.com/ros-pybullet/ros_pybullet_interface

## Reference

- Relaxed-IK Unity: https://github.com/uwgraphics/relaxed_ik_unity
- 
