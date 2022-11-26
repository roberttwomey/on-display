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
  - in one step (assuming an ros noetic install on an x64 machine)
    - `rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D /tmp/xarm7.urdf ikfast_plan link_base link_eef`
    - `rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D /tmp/$MYROBOT.urdf <planning_group_name> <base_link> <eef_link>`

- python bindings for IKFast: https://pypi.org/project/ikfast-pybind/
- from this project: https://github.com/compas-dev/compas_fab
- and this project: https://github.com/yijiangh/choreo

#### All-in-one pyikfast with docker: 
- https://github.com/cyberbotics/pyikfast
- get the docker image: `docker pull cyberbotics/pyikfast`
  - https://hub.docker.com/r/cyberbotics/pyikfast
- download xarm7.urdf as robot.urdf
- change to that directory
- generate the files: `docker run -v ${PWD}:/output cyberbotics/pyikfast link_base link_eef`
  - generate with extension: `docker run -v ${PWD}:/output cyberbotics/pyikfast link_base link_eef xarm7`
- results in [xarm7_ikfast_results.zip](https://github.com/roberttwomey/on-display-code/files/10097122/xarm7_ikfast_results.zip) linked below. 
- to compile into an executable: `make ikfast`

**Testing**
- `python3 -c "import pyikfast; print(pyikfast.forward([0.927295218001612, -2.899331265288886, 2.048719302774242, -1.057447868999410, 1.163951188044116, 0.612010251709654]))"`
- `python3 -c "import pyikfast; print(pyikfast.forward([0, 0, 0, 0, 0, 0, 0]))"`
- `python3 -c "import pyikfast; print(pyikfast.inverse([0.5, 0.5, 0.5], [1, 0, 0, 0, 1, 0, 0, 0, 1]))"`
### Forward Kinematics Test

forward kinematics for joint angles (same as look-forward-tool.py): 
`python3 -c "import pyikfast; print(pyikfast.forward([0, -0.7853981634, 0, 0, 0, -0.7853981634, 0, 0]))"`
 
results in: `[[0.2239256672224522, 0.0, 0.3999220958863359], [-5.1032511549919946e-12, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 5.1032511549919946e-12]]`
 
### Inverse Kinematics Test
```
xarm7_ikfast_results ./ikfast -5.1032511549919946e-12 0.0 1.0 0.2239256672224522 0.0 -1.0 0.0 0.0 1.0 0.0 5.1032511549919946e-12 0.3999220958863359
0
Found 8 ik solutions:
sol0 (free=0): 0.000000000000012, 2.729233667963582, 0.000000000000006, -0.799658698714147, -0.000000000000004, 1.183496613701858, 0.000000000000000, 
sol1 (free=0): -3.141592653589788, -2.729233667963582, 3.141592653589796, -0.799658698714147, -0.000000000000004, 1.183496613701858, 0.000000000000000, 
sol2 (free=0): -3.141592653589781, 0.430799325784527, -0.000000000000016, -0.799658698714147, 3.141592653589790, -0.340338302301330, 0.000000000000000, 
sol3 (free=0): 0.000000000000009, -0.430799325784527, 3.141592653589774, -0.799658698714147, 3.141592653589790, -0.340338302301330, 0.000000000000000, 
sol4 (free=0): 0.000000000000000, 3.083832505579055, 3.141592653589797, -0.000000000000003, 3.141592653589797, 1.628556474800535, 0.000000000000000, 
sol5 (free=0): 3.141592653589790, -3.083832505579055, -0.000000000000004, -0.000000000000003, 3.141592653589797, 1.628556474800535, 0.000000000000000, 
sol6 (free=0): -3.141592653589797, 0.785398163399999, 3.141592653589797, -0.000000000000003, 0.000000000000000, -0.785398163400007, 0.000000000000000, 
sol7 (free=0): -0.000000000000007, -0.785398163399999, -0.000000000000004, -0.000000000000003, 0.000000000000000, -0.785398163400007, 0.000000000000000,
```

#### CompasFab
- PyBullet inverse kinematics: https://gramaziokohler.github.io/compas_fab/latest/examples/05_backends_pybullet/02_forward_and_inverse_kinematics.html
- 
# Files:
- [xarm7.urdf.zip](https://github.com/roberttwomey/on-display-code/files/10096583/xarm7.urdf.zip)
- [xarm7_ikfast_results.zip](https://github.com/roberttwomey/on-display-code/files/10097122/xarm7_ikfast_results.zip)
