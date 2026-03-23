// Copyright (c) 2024 Pal Robotics S.L. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <mutex>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "realsense_gazebo_plugin/RealSensePlugin.hpp"

namespace realsense_gazebo_plugin
{

class GazeboRosRealsense : public RealSensePlugin
{
public:
  GazeboRosRealsense();
  ~GazeboRosRealsense() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void OnNewDepthFrame(const gz::msgs::Image & _msg) override;
  void OnNewColorFrame(const gz::msgs::Image & _msg) override;
  void OnNewInfrared1Frame(const gz::msgs::Image & _msg) override;
  void OnNewInfrared2Frame(const gz::msgs::Image & _msg) override;

  bool FillPointCloudHelper(
    sensor_msgs::msg::PointCloud2 & point_cloud_msg, uint32_t rows_arg,
    uint32_t cols_arg, uint32_t step_arg, const void * data_arg,
    const sensor_msgs::msg::Image & color_msg);

protected:
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pctnode_;

  image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;
  point_cloud_transport::Publisher pointcloud_pub_;

  sensor_msgs::msg::Image depth_msg_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;

  std::mutex color_mutex_;
  sensor_msgs::msg::Image latest_color_msg_;
};
}  // namespace realsense_gazebo_plugin