# Intel RealSense Gazebo ROS plugin

This package is a Gazebo ROS plugin for the Intel D435 realsense camera.
 
## Acknowledgement

This is a continuation of work done by [SyrianSpock](https://github.com/SyrianSpock) for a Gazebo ROS plugin with RS200 camera.

This package also includes the work developed by Intel Corporation with the ROS model fo the [D435](https://github.com/intel-ros/realsense) camera.

## Quick start

Here we provide an exmaple of using this plugin with realsense D435 camera.

First compile the package.

You need to get ```_d435.urdf.xacro```, ```_d435.gazebo.xacro```, and ```test_d435_camera.urdf.xacro``` from [realsense2_description/urdf](https://github.com/pal-robotics-forks/realsense/tree/upstream/realsense2_description/urdf)

Now do the following modifications:

1. For ```test_d435_camera.urdf.xacro```, modify line 3 (directory for _d435.urdf.xacro) according to your directory.
2. For ```_d435.urdf.xacro```, modify line 13 (directory for _d435.gazebo.xacro) according to your directory.
3. If you **DO NOT** have [realsense-ros](https://github.com/IntelRealSense/realsense-ros) installed, in ```_d435.urdf.xacro```, you can uncomment line 61 and comment line 62 in _d435.urdf.xacro to get rid of the mesh usage from realsense-ros. <br>
_Note: if you used apt-get to install the realsense-ros, you also need to install ros-$ROS_DISTRO-realsense2-description separately to get the mesh._

Now launch the terminal **in the directory where you put these .xacro files**.

```
rosrun xacro xacro test_d435_camera.urdf.xacro > test_d435_camera.urdf
rosrun gazebo_ros gazebo
// make sure you have the right directory for "rosrun gazebo_ros gazebo"! Or it may not be working.
```

And in another terminal:

```
rosrun spawn_model -urdf -file test_d435_camera.urdf -model realsense2_camera
```

Gazebo should be running with a d435 camera (tested on noetic and melodic).

For adding the camera to your .urdf / .xacro file, please look into ```test_d435_camera.urdf.xacro``` and ```test_d435_camera.urdf```.
