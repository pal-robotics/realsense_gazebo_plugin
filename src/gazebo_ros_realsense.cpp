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
#include <limits>

#include <gz/plugin/Register.hh>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#define DEPTH_SCALE_M 0.001

GZ_ADD_PLUGIN(
  realsense_gazebo_plugin::GazeboRosRealsense,
  gz::sim::System,
  realsense_gazebo_plugin::GazeboRosRealsense::ISystemConfigure)

namespace
{
sensor_msgs::msg::CameraInfo cameraInfo(
  const sensor_msgs::msg::Image & image, float horizontal_fov)
{
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header = image.header;
  info_msg.distortion_model = "plumb_bob";
  info_msg.height = image.height;
  info_msg.width = image.width;

  const float focal = 0.5f * image.width / tan(0.5f * horizontal_fov);

  info_msg.k[0] = focal;
  info_msg.k[4] = focal;
  info_msg.k[2] = info_msg.width * 0.5;
  info_msg.k[5] = info_msg.height * 0.5;
  info_msg.k[8] = 1.;

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
  RCLCPP_DEBUG_STREAM(this->node_->get_logger(), "realsense_camera Unloaded");
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

  this->color_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), prefix + std::string("/") +
    cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);
  this->ir1_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), prefix + std::string("/") +
    cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);
  this->ir2_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), prefix + std::string("/") +
    cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);
  this->depth_pub_ = image_transport::create_camera_publisher(
    this->node_.get(), prefix + std::string("/") +
    cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, rmw_qos_profile_sensor_data);

  if (pointCloud_) {
    this->pctnode_ = std::make_unique<point_cloud_transport::PointCloudTransport>(this->node_);
    this->pointcloud_pub_ = this->pctnode_->advertise(
      prefix + std::string("/") + pointCloudTopic_, rmw_qos_profile_sensor_data);
  }

  RCLCPP_INFO(node_->get_logger(), "Loaded Realsense Gazebo ROS plugin.");
}

void GazeboRosRealsense::OnNewColorFrame(const gz::msgs::Image & _msg)
{
  this->image_msg_.header.frame_id = cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame;
  this->image_msg_.header.stamp = node_->now();
  this->image_msg_.height = _msg.height();
  this->image_msg_.width = _msg.width();
  this->image_msg_.encoding = sensor_msgs::image_encodings::RGB8;
  this->image_msg_.is_bigendian = false;
  this->image_msg_.step = _msg.width() * 3;
  this->image_msg_.data.resize(this->image_msg_.step * this->image_msg_.height);
  std::copy(_msg.data().begin(), _msg.data().end(), this->image_msg_.data.begin());

  auto info_msg = cameraInfo(this->image_msg_, cameraParamsMap_[COLOR_CAMERA_NAME].hfov);
  color_pub_.publish(this->image_msg_, info_msg);
}

void GazeboRosRealsense::OnNewInfrared1Frame(const gz::msgs::Image & _msg)
{
  this->image_msg_.header.frame_id = cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame;
  this->image_msg_.header.stamp = node_->now();
  this->image_msg_.height = _msg.height();
  this->image_msg_.width = _msg.width();
  this->image_msg_.encoding = sensor_msgs::image_encodings::MONO8;
  this->image_msg_.is_bigendian = false;
  this->image_msg_.step = _msg.width();
  this->image_msg_.data.resize(this->image_msg_.step * this->image_msg_.height);
  std::copy(_msg.data().begin(), _msg.data().end(), this->image_msg_.data.begin());

  auto info_msg = cameraInfo(this->image_msg_, cameraParamsMap_[IRED1_CAMERA_NAME].hfov);
  ir1_pub_.publish(this->image_msg_, info_msg);
}

void GazeboRosRealsense::OnNewInfrared2Frame(const gz::msgs::Image & _msg)
{
  this->image_msg_.header.frame_id = cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame;
  this->image_msg_.header.stamp = node_->now();
  this->image_msg_.height = _msg.height();
  this->image_msg_.width = _msg.width();
  this->image_msg_.encoding = sensor_msgs::image_encodings::MONO8;
  this->image_msg_.is_bigendian = false;
  this->image_msg_.step = _msg.width();
  this->image_msg_.data.resize(this->image_msg_.step * this->image_msg_.height);
  std::copy(_msg.data().begin(), _msg.data().end(), this->image_msg_.data.begin());

  auto info_msg = cameraInfo(this->image_msg_, cameraParamsMap_[IRED2_CAMERA_NAME].hfov);
  ir2_pub_.publish(this->image_msg_, info_msg);
}

void GazeboRosRealsense::OnNewDepthFrame(const gz::msgs::Image & _msg)
{
  this->depth_msg_.header.frame_id = cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame;
  this->depth_msg_.header.stamp = node_->now();
  this->depth_msg_.height = _msg.height();
  this->depth_msg_.width = _msg.width();
  this->depth_msg_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  this->depth_msg_.is_bigendian = false;
  this->depth_msg_.step = _msg.width() * 2;
  this->depth_msg_.data.resize(this->depth_msg_.step * this->depth_msg_.height);

  unsigned int imageSize = _msg.width() * _msg.height();
  const float * depthDataFloat = reinterpret_cast<const float *>(_msg.data().data());
  uint16_t * depthDataUint = reinterpret_cast<uint16_t *>(this->depth_msg_.data.data());

  for (unsigned int i = 0; i < imageSize; ++i) {
    if (depthDataFloat[i] < rangeMinDepth_ ||
      depthDataFloat[i] > rangeMaxDepth_ ||
      depthDataFloat[i] < 0)
    {
      depthDataUint[i] = 0;
    } else {
      depthDataUint[i] = static_cast<uint16_t>(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  auto depth_info_msg = cameraInfo(this->depth_msg_, cameraParamsMap_[DEPTH_CAMERA_NAME].hfov);
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);

  if (pointCloud_ && this->pointcloud_pub_.getNumSubscribers() > 0) {
    this->pointcloud_msg_.header = this->depth_msg_.header;
    FillPointCloudHelper(
      this->pointcloud_msg_, _msg.height(), _msg.width(),
      _msg.width() * sizeof(float), depthDataFloat);
    this->pointcloud_pub_.publish(this->pointcloud_msg_);
  }
}

bool GazeboRosRealsense::FillPointCloudHelper(
  sensor_msgs::msg::PointCloud2 & point_cloud_msg, uint32_t rows_arg,
  uint32_t cols_arg, uint32_t /*step_arg*/, const void * data_arg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(rows_arg * cols_arg);
  point_cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg, "rgb");

  const float * toCopyFrom = reinterpret_cast<const float *>(data_arg);
  size_t index = 0;

  const double hfov = cameraParamsMap_[DEPTH_CAMERA_NAME].hfov;
  const double focal = static_cast<double>(cols_arg) / (2.0 * tan(hfov / 2.0));

  for (uint32_t j = 0; j < rows_arg; j++) {
    double pAngle;
    if (rows_arg > 1) {
      pAngle = atan2(
        static_cast<double>(j) - 0.5 * static_cast<double>(rows_arg - 1), focal);
    } else {
      pAngle = 0.0;
    }

    for (uint32_t i = 0; i < cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
      const double depth = toCopyFrom[index++];

      if (std::isfinite(depth) && depth > pointCloudCutOff_ && depth < pointCloudCutOffMax_) {
        double yAngle;
        if (cols_arg > 1) {
          yAngle = atan2(
            static_cast<double>(i) - 0.5 * static_cast<double>(cols_arg - 1), focal);
        } else {
          yAngle = 0.0;
        }

        *iter_x = depth * tan(yAngle);
        *iter_y = depth * tan(pAngle);
        *iter_z = depth;
      } else {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        point_cloud_msg.is_dense = false;
      }

      uint8_t * image_src = reinterpret_cast<uint8_t *>(this->image_msg_.data.data());
      if (this->image_msg_.data.size() == static_cast<size_t>(rows_arg) * cols_arg * 3) {
        if (this->image_msg_.encoding == sensor_msgs::image_encodings::RGB8) {
          iter_rgb[2] = image_src[i * 3 + j * cols_arg * 3 + 0];
          iter_rgb[1] = image_src[i * 3 + j * cols_arg * 3 + 1];
          iter_rgb[0] = image_src[i * 3 + j * cols_arg * 3 + 2];
        } else if (this->image_msg_.encoding == sensor_msgs::image_encodings::BGR8) {
          iter_rgb[0] = image_src[i * 3 + j * cols_arg * 3 + 0];
          iter_rgb[1] = image_src[i * 3 + j * cols_arg * 3 + 1];
          iter_rgb[2] = image_src[i * 3 + j * cols_arg * 3 + 2];
        } else {
          throw std::runtime_error(
            "unsupported colour encoding: " + this->image_msg_.encoding);
        }
      } else if (this->image_msg_.data.size() == static_cast<size_t>(rows_arg) * cols_arg) {
        iter_rgb[0] = image_src[i + j * cols_arg];
        iter_rgb[1] = image_src[i + j * cols_arg];
        iter_rgb[2] = image_src[i + j * cols_arg];
      } else {
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;
  return true;
}

}  // namespace realsense_gazebo_plugin
