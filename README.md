# Intel RealSense Gazebo ROS plugin

This package provides a Gazebo Harmonic (`gz-sim`) ROS 2 plugin for the Intel D435 RealSense camera.

## Note

This branch targets ROS 2 Jazzy with Gazebo Harmonic (`gz-sim 8`). If you need a Gazebo Classic setup, use the branch that still depends on `gazebo_ros_pkgs`.

## Example usage with a custom robot

In Gazebo Harmonic, simulated sensors publish over native gz-transport topics. This plugin must be told which native sensor topics to subscribe to, and it also needs the configured horizontal field of view values so it can generate ROS camera info messages.

Use `gz topic -l` to find the exact sensor topic paths for your model.

```xml
<plugin name="realsense_gazebo_plugin::GazeboRosRealsense" filename="librealsense_gazebo_plugin.so">
  <prefix>camera</prefix>
  <depthUpdateRate>30.0</depthUpdateRate>
  <colorUpdateRate>30.0</colorUpdateRate>
  <infraredUpdateRate>1.0</infraredUpdateRate>

  <!-- Gazebo Harmonic native topics to subscribe to -->
  <gzDepthTopic>/world/default/model/chiconybot/link/realsense_link/sensor/cameradepth/depth_image</gzDepthTopic>
  <gzColorTopic>/world/default/model/chiconybot/link/realsense_link/sensor/cameracolor/image</gzColorTopic>
  <gzInfrared1Topic>/world/default/model/chiconybot/link/realsense_link/sensor/cameraired1/image</gzInfrared1Topic>
  <gzInfrared2Topic>/world/default/model/chiconybot/link/realsense_link/sensor/cameraired2/image</gzInfrared2Topic>

  <depthHFOV>1.57</depthHFOV>
  <colorHFOV>1.57</colorHFOV>
  <infraredHFOV>1.57</infraredHFOV>

  <!-- ROS topic configuration -->
  <depthTopicName>aligned_depth_to_color/image_raw</depthTopicName>
  <depthCameraInfoTopicName>depth/camera_info</depthCameraInfoTopicName>
  <colorTopicName>color/image_raw</colorTopicName>
  <colorCameraInfoTopicName>color/camera_info</colorCameraInfoTopicName>
  <infrared1TopicName>infra1/image_raw</infrared1TopicName>
  <infrared1CameraInfoTopicName>infra1/camera_info</infrared1CameraInfoTopicName>
  <infrared2TopicName>infra2/image_raw</infrared2TopicName>
  <infrared2CameraInfoTopicName>infra2/camera_info</infrared2CameraInfoTopicName>
  <colorOpticalframeName>camera_color_optical_frame</colorOpticalframeName>
  <depthOpticalframeName>camera_depth_optical_frame</depthOpticalframeName>
  <infrared1OpticalframeName>camera_left_ir_optical_frame</infrared1OpticalframeName>
  <infrared2OpticalframeName>camera_right_ir_optical_frame</infrared2OpticalframeName>
  <rangeMinDepth>0.3</rangeMinDepth>
  <rangeMaxDepth>3.0</rangeMaxDepth>
  <pointCloud>true</pointCloud>
  <pointCloudTopicName>depth/color/points</pointCloudTopicName>
  <pointCloudCutoff>0.3</pointCloudCutoff>
</plugin>
```
