# Intel RealSense Gazebo ROS plugin and model

## _**Important note**_

**This branch only supports ROS Melodic with Gazebo 9.**

**To use this plugin with ROS Kinetic and Gazebo 7 you should use the [`kinetic-devel` branch](https://github.com/SyrianSpock/realsense_gazebo_plugin/tree/kinetic-devel).**

## Quickstart

Build the plugin
```bash
catkin build realsense_gazebo_plugin
```

Test it by running
```bash
roslaunch realsense_gazebo_plugin realsense.launch
```

## Run the unittests

After building the plugin, you can run the unittests
```bash
rostest realsense_gazebo_plugin realsense_streams.test
```

## Run the point cloud demo

Using [depth_image_proc](http://wiki.ros.org/depth_image_proc) package, we can generate a point cloud from the depth image by running
```bash
roslaunch realsense_gazebo_plugin realsense.launch # in terminal 1
roslaunch realsense_gazebo_plugin depth_proc.launch # in terminal 2
```

Then open Rviz, and display the `/realsense/camera/depth_registered/points` topic, you should see something like this
![Point cloud in Rviz](doc/pointcloud.png)

## Run from URDF

```bash
roslaunch realsense_gazebo_plugin realsense_urdf.launch
```

This will behave the same as `realsense.launch` mentioned above, with the difference that it spawns the model from a URDF (see `urdf` folder).
You can reuse this to plug the sensor in the robot of your choice.

## Dependencies

This requires Gazebo 6 or higher and catkin tools for building.

The package has been tested on ROS melodic on Ubuntu 18.04 with Gazebo 9.

## Acknowledgement

This is continuation of work done by [guiccbr](https://github.com/guiccbr/) for Intel Corporation.

Thanks to [Danfoa](https://github.com/Danfoa) for contributing the URDF integration.
