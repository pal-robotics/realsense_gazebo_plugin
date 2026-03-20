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

#include "realsense_gazebo_plugin/gazebo_ros_realsense.hpp"
#include <cmath>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <gz/plugin/Register.hh>

#define DEPTH_SCALE_M 0.001

GZ_ADD_PLUGIN(
  realsense_gazebo_plugin::GazeboRosRealsense,
  gz::sim::System,
  realsense_gazebo_plugin::GazeboRosRealsense::ISystemConfigure)

namespace
{
rclcpp::Time getRosStamp(const gz::msgs::Image & image_msg, const rclcpp::Node::SharedPtr & node)
{
  // Use Gazebo's message timestamp so simulated camera data stays in sim time.
  // Falling back to node time keeps behavior safe if a header stamp is absent.
  const auto & header = image_msg.header();
  if (header.has_stamp()) {
    const auto & stamp = header.stamp();
    const auto stamp_ns =
      static_cast<int64_t>(stamp.sec()) * 1000000000LL + static_cast<int64_t>(stamp.nsec());
    return rclcpp::Time(stamp_ns, RCL_ROS_TIME);
  }

  return node->now();
}

sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image & image, float horizontal_fov)
{
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header = image.header;
  info_msg.distortion_model = "plumb_bob";
  info_msg.height = image.height;
  info_msg.width = image.width;
  const double focal = 0.5 * static_cast<double>(image.width) / tan(0.5 * horizontal_fov);
  const double cx = 0.5 * static_cast<double>(image.width - 1);
  const double cy = 0.5 * static_cast<double>(image.height - 1);
  info_msg.k[0] = focal;
  info_msg.k[4] = focal;
  info_msg.k[2] = cx;
  info_msg.k[5] = cy;
  info_msg.k[8] = 1.;
  info_msg.r[0] = 1.;
  info_msg.r[4] = 1.;
  info_msg.r[8] = 1.;
  info_msg.p[0] = info_msg.k[0];
  info_msg.p[5] = info_msg.k[4];
  info_msg.p[2] = info_msg.k[2];
  info_msg.p[6] = info_msg.k[5];
  info_msg.p[10] = info_msg.k[8];
  return info_msg;
}
}  // namespace

namespace realsense_gazebo_plugin
{

GazeboRosRealsense::GazeboRosRealsense() {}

GazeboRosRealsense::~GazeboRosRealsense()
{
  if (node_) {
    RCLCPP_DEBUG_STREAM(this->node_->get_logger(), "realsense_camera Unloaded");
  }
}

void GazeboRosRealsense::Configure(const gz::sim::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &_ecm,
                                   gz::sim::EventManager &_eventMgr)
{
  RealSensePlugin::Configure(_entity, _sdf, _ecm, _eventMgr);

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::string node_name = "gazebo_realsense";
  node_name += this->prefix.empty() ? "" : "_" + this->prefix;
  this->node_ = rclcpp::Node::make_shared(node_name);

  RCLCPP_INFO(node_->get_logger(), "Realsense Gazebo ROS plugin loading.");

  this->camera_info_manager_.reset(
    new camera_info_manager::CameraInfoManager(this->node_.get(), node_name));

  // Helper lambda to gracefully parse absolute (leading '/') ROS topic names
  // without creating double slashes with the node's prefix string.
  auto build_topic = [](const std::string& pfx, const std::string& tpc) {
    if (tpc.empty()) return pfx;
    if (tpc.front() == '/') return tpc; // Absolute topic, return as-is
    return pfx.empty() ? tpc : pfx + "/" + tpc;
  };

  this->color_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), build_topic(this->prefix, cameraParamsMap_[COLOR_CAMERA_NAME].topic_name), rmw_qos_profile_sensor_data);
  this->ir1_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), build_topic(this->prefix, cameraParamsMap_[IRED1_CAMERA_NAME].topic_name), rmw_qos_profile_sensor_data);
  this->ir2_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), build_topic(this->prefix, cameraParamsMap_[IRED2_CAMERA_NAME].topic_name), rmw_qos_profile_sensor_data);
  this->depth_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), build_topic(this->prefix, cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name), rmw_qos_profile_sensor_data);

  if (pointCloud_) {
    this->pctnode_ = std::make_unique<point_cloud_transport::PointCloudTransport>(this->node_);
    this->pointcloud_pub_ = this->pctnode_->advertise(
      build_topic(this->prefix, pointCloudTopic_), rmw_qos_profile_sensor_data);
  }

  RCLCPP_INFO(node_->get_logger(), "Loaded Realsense Gazebo ROS plugin.");
}

void GazeboRosRealsense::OnNewColorFrame(const gz::msgs::Image & _msg)
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame;
  msg.header.stamp = getRosStamp(_msg, node_);
  msg.height = _msg.height();
  msg.width = _msg.width();
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  msg.is_bigendian = false;
  msg.step = _msg.width() * 3;
  msg.data.resize(msg.step * msg.height);
  std::copy(_msg.data().begin(), _msg.data().end(), msg.data.begin());

  auto info_msg = cameraInfo(msg, cameraParamsMap_[COLOR_CAMERA_NAME].hfov);
  color_pub_.publish(msg, info_msg);

  {
    std::lock_guard<std::mutex> lock(color_mutex_);
    latest_color_msg_ = msg;
  }
}

void GazeboRosRealsense::OnNewInfrared1Frame(const gz::msgs::Image & _msg)
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame;
  msg.header.stamp = getRosStamp(_msg, node_);
  msg.height = _msg.height();
  msg.width = _msg.width();
  msg.encoding = sensor_msgs::image_encodings::MONO8;
  msg.is_bigendian = false;
  msg.step = _msg.width();
  msg.data.resize(msg.step * msg.height);
  std::copy(_msg.data().begin(), _msg.data().end(), msg.data.begin());

  auto info_msg = cameraInfo(msg, cameraParamsMap_[IRED1_CAMERA_NAME].hfov);
  ir1_pub_.publish(msg, info_msg);
}

void GazeboRosRealsense::OnNewInfrared2Frame(const gz::msgs::Image & _msg)
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame;
  msg.header.stamp = getRosStamp(_msg, node_);
  msg.height = _msg.height();
  msg.width = _msg.width();
  msg.encoding = sensor_msgs::image_encodings::MONO8;
  msg.is_bigendian = false;
  msg.step = _msg.width();
  msg.data.resize(msg.step * msg.height);
  std::copy(_msg.data().begin(), _msg.data().end(), msg.data.begin());

  auto info_msg = cameraInfo(msg, cameraParamsMap_[IRED2_CAMERA_NAME].hfov);
  ir2_pub_.publish(msg, info_msg);
}

void GazeboRosRealsense::OnNewDepthFrame(const gz::msgs::Image & _msg)
{
  this->depth_msg_.header.frame_id = cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame;
  this->depth_msg_.header.stamp = getRosStamp(_msg, node_);
  this->depth_msg_.height = _msg.height();
  this->depth_msg_.width = _msg.width();
  this->depth_msg_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  this->depth_msg_.is_bigendian = false;
  this->depth_msg_.step = _msg.width() * 2;
  this->depth_msg_.data.resize(this->depth_msg_.step * this->depth_msg_.height);

  unsigned int imageSize = _msg.width() * _msg.height();
  const float * depthDataFloat = reinterpret_cast<const float*>(_msg.data().c_str());
  uint16_t * depthDataUint = reinterpret_cast<uint16_t*>(this->depth_msg_.data.data());

  for (unsigned int i = 0; i < imageSize; ++i) {
    if (depthDataFloat[i] < rangeMinDepth_ || depthDataFloat[i] > rangeMaxDepth_ || depthDataFloat[i] < 0) {
      depthDataUint[i] = 0;
    } else {
      depthDataUint[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  auto depth_info_msg = cameraInfo(this->depth_msg_, cameraParamsMap_[DEPTH_CAMERA_NAME].hfov);
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);

  if (pointCloud_ && this->pointcloud_pub_.getNumSubscribers() > 0) {
    this->pointcloud_msg_.header = this->depth_msg_.header;

    sensor_msgs::msg::Image color_msg_copy;
    {
       std::lock_guard<std::mutex> lock(color_mutex_);
       color_msg_copy = latest_color_msg_;
    }

    FillPointCloudHelper(
      this->pointcloud_msg_, _msg.height(), _msg.width(),
      _msg.width() * sizeof(float), depthDataFloat, color_msg_copy);
    this->pointcloud_pub_.publish(this->pointcloud_msg_);
  }
}

bool GazeboRosRealsense::FillPointCloudHelper(
  sensor_msgs::msg::PointCloud2 & point_cloud_msg, uint32_t rows_arg,
  uint32_t cols_arg, uint32_t /*step_arg*/, const void * data_arg,
  const sensor_msgs::msg::Image & color_msg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(cols_arg, rows_arg);
  point_cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg, "rgb");

  const float * toCopyFrom = reinterpret_cast<const float *>(data_arg);
  size_t index = 0;

  const double hfov = cameraParamsMap_[DEPTH_CAMERA_NAME].hfov;
  const double focal = static_cast<double>(cols_arg) / (2.0 * tan(hfov / 2.0));
  const double cx = 0.5 * static_cast<double>(cols_arg - 1);
  const double cy = 0.5 * static_cast<double>(rows_arg - 1);

  const bool has_color = !color_msg.data.empty();
  const bool color_dimensions_match =
    has_color &&
    color_msg.encoding == sensor_msgs::image_encodings::RGB8 &&
    color_msg.width == cols_arg &&
    color_msg.height == rows_arg &&
    color_msg.step >= cols_arg * 3 &&
    color_msg.data.size() >= static_cast<size_t>(color_msg.step) * color_msg.height;

  if (has_color && !color_dimensions_match) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "Skipping point cloud colorization because the color frame (%ux%u, encoding=%s) does not "
      "match the depth frame (%ux%u).",
      color_msg.width, color_msg.height, color_msg.encoding.c_str(), cols_arg, rows_arg);
  }

  for (uint32_t j = 0; j < rows_arg; j++) {
    for (uint32_t i = 0; i < cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
      const double depth = toCopyFrom[index++];

      if (std::isfinite(depth) && depth > pointCloudCutOff_ && depth < pointCloudCutOffMax_) {
        const double normalized_x = (static_cast<double>(i) - cx) / focal;
        const double normalized_y = (static_cast<double>(j) - cy) / focal;

        *iter_x = depth * normalized_x;
        *iter_y = depth * normalized_y;
        *iter_z = depth;
      } else {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        point_cloud_msg.is_dense = false;
      }

      if (color_dimensions_match) {
        const auto pixel_offset = static_cast<size_t>(j) * color_msg.step + static_cast<size_t>(i) * 3U;
        iter_rgb[2] = color_msg.data[pixel_offset + 0];
        iter_rgb[1] = color_msg.data[pixel_offset + 1];
        iter_rgb[0] = color_msg.data[pixel_offset + 2];
      } else {
        iter_rgb[0] = iter_rgb[1] = iter_rgb[2] = 0;
      }
    }
  }

  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;
  return true;
}

}  // namespace realsense_gazebo_plugin
