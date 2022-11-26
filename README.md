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
- xArm7-CPP_SDK: https://github.com/xArm-Developer/xArm-CPLUS-SDK
  - builds fine on mac M1. run examples from `/Users/rtwomey/code/xArm-CPLUS-SDK/`. 
  - shared libraries is in `/Users/rtwomey/code/xArm-CPLUS-SDK/build/lib/libxarm.so`
  
### IK Fast
- http://docs.ros.org/en/kinetic/api/framefab_irb6600_support/html/doc/ikfast_tutorial.html
- UR IK Fast python https://github.com/cambel/ur_ikfast
- https://www.oreilly.com/library/view/mastering-ros-for/9781783551798/ch11s12.html
- http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution
  - Installing OpenRave on 20.04 https://robots.uc3m.es/installation-guides/install-openrave.html#install-openrave-via-scripts-ubuntu-1804-bionic-and-ubuntu-2004-focal
  - **No Need to install from scratch**: https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html#getting-started
  - in one step
    - `rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D /tmp/xarm7.urdf <planning_group_name> link_base link_eef`
    - `rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D /tmp/$MYROBOT.urdf <planning_group_name> <base_link> <eef_link>`

[xarm7.urdf.zip](https://github.com/roberttwomey/on-display-code/files/10096583/xarm7.urdf.zip)
