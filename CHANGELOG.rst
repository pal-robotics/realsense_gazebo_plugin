^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_gazebo_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2025-07-03)
------------------
* typo and removed useless pub
* using sensor_data QoS for all publishers
* Contributors: andreacapodacqua

3.1.0 (2025-01-23)
------------------
* Merge branch 'abr/feat/pc-transport' into 'alum-devel'
  use point_cloud_transport
  See merge request device/realsense_gazebo_plugin!20
* use point_cloud_transport
* Contributors: antoniobrandi, sergiomoyano

3.0.2 (2024-07-02)
------------------
* Merge branch 'man/feat/image-qos-best-effort' into 'alum-devel'
  Man/feat/image qos best effort
  See merge request device/realsense_gazebo_plugin!19
* changed qos profile for camera color pub
* using sensor data QoS
* Contributors: andreacapodacqua, martinaannicelli, sergiomoyano

3.0.1 (2024-02-27)
------------------
* Merge branch 'smd/feat/topic_prefix' into 'alum-devel'
  Added prefix to all the topics published by the plugin
  See merge request device/realsense_gazebo_plugin!17
* Added prefix to all the topics published by the plugin
* Contributors: sergiomoyano

3.0.0 (2024-02-21)
------------------
* Merge branch 'new_maintainer' into 'alum-devel'
  Added new maintainer sergio
  See merge request device/realsense_gazebo_plugin!16
* Added build type ament_cmake
* Setting the types right for data_arg in FillPointCloudHelper
* Fix ament linters errors
* Added test depend ament_lint
* Added new maintainer sergio
* Merge pull request #41 from Maidbot/foxy-devel
  [Bug-fix] Old lines preventing build
* Merge pull request #4 from Maidbot/bug-fix/memory-leak-image-transport
  [Bug-fix] Image transport memory leak
* Make image transport a smart pointer so it gets freed when gazebo_ros_realsense plugin is deleted
* Merge pull request #3 from Maidbot/bug-fix/rosnode-stdcout
  [Bug-fix] Remove rosnode and stdcout.
* Remove rosnode preventing build. Remove std::cout in favor of rclcpp logging
* Merge pull request #39 from jediofgever/ros2
  Add ros2 support
* Merge branch 'foxy-devel' into ros2
* added more comments on the png compression format for depth images
* Merge pull request #28 from christian-rauch/depth_compressed_png
  set 'png' compression format for depth images
* set 'png' compression format for depth images
  By default the compressed_image_transport is using the jpeg format. This is
  a lossy compression format which introduces compression artefacts which are
  not visible in colour images, but corrupt the 16bit depth images.
  Change the depth compression to the lossless png format to keep the original
  data when decompressing depth images.
* Merge pull request #30 from christian-rauch/colour_channel_fix
  fix colour channel order for point cloud
* fix colour channel order for point cloud
* update readme
* add how to use to readme
* update readme
* add ros2 support
* Merge pull request #16 from dvigne/melodic-devel
  Changed Distortion Model in Camera Info Messages
* Changed Distortion Model in Camera Info Messages
* Contributors: Christian Rauch, Derick Vigne, Fetullah Atas, Sai Kishor Kothakota, Victor Lopez, jediofgever, john-maidbot, sergiomoyano

1.1.0 (2020-01-30)
------------------
* Merge branch 'pointcloud_ferrum' into 'ferrum-devel'
  added methods to publish the pointcloud information
  See merge request device/realsense_gazebo_plugin!14
* added methods to publish the pointcloud information
* Update readme to remove explicit mention to REEM-C
* Contributors: Sai Kishor Kothakota, Victor Lopez

1.0.4 (2019-12-10)
------------------
* Merge branch 'gazebo_xacro_ferrum' into 'ferrum-devel'
  remove gazebo and URDF xacro
  See merge request device/realsense_gazebo_plugin!12
* remove gazebo and URDF xacro
* Contributors: Sai Kishor Kothakota, Victor Lopez

1.0.3 (2019-11-05)
------------------
* Add SYSTEM to include_directories
* Contributors: Victor Lopez

1.0.2 (2019-10-30)
------------------
* Merge branch 'mesh-fix-ferrum' into 'ferrum-devel'
  fixed the issue with mesh location
  See merge request device/realsense_gazebo_plugin!10
* fixed the issue with mesh location
* Contributors: Adria Roig, Sai Kishor Kothakota

1.0.1 (2019-10-30)
------------------
* Merge branch 'multi-realsense-ferrum' into 'ferrum-devel'
  Update plugin to support multiple realsense camera's
  See merge request device/realsense_gazebo_plugin!7
* Update plugin to support multiple realsense camera's
* Contributors: Adria Roig, Sai Kishor Kothakota

1.0.0 (2019-09-10)
------------------
* Adapted to latest gazebo API changes
* Contributors: Jordan Palacios

0.0.3 (2019-03-28)
------------------
* Fix licenses for public release
* Contributors: Adria Roig

0.0.2 (2019-03-05)
------------------
* Merge branch 'realsense' into 'master'
  Fix missing installation files
  See merge request device/realsense_gazebo_plugin!2
* Fix missing installation files
* Contributors: Adria Roig

0.0.1 (2019-03-01)
------------------
* Rm unnecessary dependees
* Fix dependencies
* Merge branch 'realsense' into 'master'
  Realsense
  See merge request adriaroig/realsense_gazebo_plugin!1
* Rm unnecessary files
* Rm unneccessary files
* Delete CMakeLists.txt.user
* Initial commit
* Contributors: Adria Roig
