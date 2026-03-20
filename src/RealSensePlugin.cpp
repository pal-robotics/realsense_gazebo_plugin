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

#include "realsense_gazebo_plugin/RealSensePlugin.hpp"
#include <iostream>
#include <limits>

#define DEPTH_SCALE_M 0.001

namespace realsense_gazebo_plugin
{

RealSensePlugin::RealSensePlugin()
: pointCloudCutOff_(0.0),
  pointCloudCutOffMax_(5.0),
  colorUpdateRate_(0.0),
  infraredUpdateRate_(0.0),
  depthUpdateRate_(0.0),
  rangeMinDepth_(0.0f),
  rangeMaxDepth_(std::numeric_limits<float>::max())
{}

RealSensePlugin::~RealSensePlugin() {}

void RealSensePlugin::Configure(const gz::sim::Entity & /*_entity*/,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager & /*_ecm*/,
                                gz::sim::EventManager & /*_eventMgr*/)
{
  cameraParamsMap_.insert(std::make_pair(COLOR_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(DEPTH_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED1_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED2_CAMERA_NAME, CameraParams()));

  if (_sdf->HasElement("depthUpdateRate")) depthUpdateRate_ = _sdf->Get<double>("depthUpdateRate");
  if (_sdf->HasElement("colorUpdateRate")) colorUpdateRate_ = _sdf->Get<double>("colorUpdateRate");
  if (_sdf->HasElement("infraredUpdateRate")) infraredUpdateRate_ = _sdf->Get<double>("infraredUpdateRate");

  if (_sdf->HasElement("depthTopicName")) cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name = _sdf->Get<std::string>("depthTopicName");
  if (_sdf->HasElement("depthCameraInfoTopicName")) cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("depthCameraInfoTopicName");
  if (_sdf->HasElement("colorTopicName")) cameraParamsMap_[COLOR_CAMERA_NAME].topic_name = _sdf->Get<std::string>("colorTopicName");
  if (_sdf->HasElement("colorCameraInfoTopicName")) cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("colorCameraInfoTopicName");
  if (_sdf->HasElement("infrared1TopicName")) cameraParamsMap_[IRED1_CAMERA_NAME].topic_name = _sdf->Get<std::string>("infrared1TopicName");
  if (_sdf->HasElement("infrared1CameraInfoTopicName")) cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("infrared1CameraInfoTopicName");
  if (_sdf->HasElement("infrared2TopicName")) cameraParamsMap_[IRED2_CAMERA_NAME].topic_name = _sdf->Get<std::string>("infrared2TopicName");
  if (_sdf->HasElement("infrared2CameraInfoTopicName")) cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name = _sdf->Get<std::string>("infrared2CameraInfoTopicName");

  if (_sdf->HasElement("colorOpticalframeName")) cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("colorOpticalframeName");
  if (_sdf->HasElement("depthOpticalframeName")) cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("depthOpticalframeName");
  if (_sdf->HasElement("infrared1OpticalframeName")) cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("infrared1OpticalframeName");
  if (_sdf->HasElement("infrared2OpticalframeName")) cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame = _sdf->Get<std::string>("infrared2OpticalframeName");

  if (_sdf->HasElement("rangeMinDepth")) rangeMinDepth_ = _sdf->Get<float>("rangeMinDepth");
  if (_sdf->HasElement("rangeMaxDepth")) rangeMaxDepth_ = _sdf->Get<float>("rangeMaxDepth");
  if (_sdf->HasElement("pointCloud")) pointCloud_ = _sdf->Get<bool>("pointCloud");
  if (_sdf->HasElement("pointCloudTopicName")) pointCloudTopic_ = _sdf->Get<std::string>("pointCloudTopicName");
  if (_sdf->HasElement("pointCloudCutoff")) pointCloudCutOff_ = _sdf->Get<double>("pointCloudCutoff");
  if (_sdf->HasElement("pointCloudCutoffMax")) pointCloudCutOffMax_ = _sdf->Get<double>("pointCloudCutoffMax");
  if (_sdf->HasElement("prefix")) this->prefix = _sdf->Get<std::string>("prefix");

  // In Gazebo Harmonic, we need to explicitly know the gz-transport topics the native cameras publish.
  if (_sdf->HasElement("gzDepthTopic")) cameraParamsMap_[DEPTH_CAMERA_NAME].gz_topic = _sdf->Get<std::string>("gzDepthTopic");
  if (_sdf->HasElement("gzColorTopic")) cameraParamsMap_[COLOR_CAMERA_NAME].gz_topic = _sdf->Get<std::string>("gzColorTopic");
  if (_sdf->HasElement("gzInfrared1Topic")) cameraParamsMap_[IRED1_CAMERA_NAME].gz_topic = _sdf->Get<std::string>("gzInfrared1Topic");
  if (_sdf->HasElement("gzInfrared2Topic")) cameraParamsMap_[IRED2_CAMERA_NAME].gz_topic = _sdf->Get<std::string>("gzInfrared2Topic");

  // Since we don't query the visual ECM renderer anymore to maintain sync, we get the known configured HFOVs
  cameraParamsMap_[DEPTH_CAMERA_NAME].hfov = _sdf->HasElement("depthHFOV") ? _sdf->Get<double>("depthHFOV") : 1.57;
  cameraParamsMap_[COLOR_CAMERA_NAME].hfov = _sdf->HasElement("colorHFOV") ? _sdf->Get<double>("colorHFOV") : 1.57;
  cameraParamsMap_[IRED1_CAMERA_NAME].hfov = _sdf->HasElement("infraredHFOV") ? _sdf->Get<double>("infraredHFOV") : 1.57;
  cameraParamsMap_[IRED2_CAMERA_NAME].hfov = cameraParamsMap_[IRED1_CAMERA_NAME].hfov;

  // Subscribe to Gazebo transport topics natively
  if (!cameraParamsMap_[DEPTH_CAMERA_NAME].gz_topic.empty()) {
    this->transportNode.Subscribe(cameraParamsMap_[DEPTH_CAMERA_NAME].gz_topic, &RealSensePlugin::OnNewDepthFrame, this);
  }
  if (!cameraParamsMap_[COLOR_CAMERA_NAME].gz_topic.empty()) {
    this->transportNode.Subscribe(cameraParamsMap_[COLOR_CAMERA_NAME].gz_topic, &RealSensePlugin::OnNewColorFrame, this);
  }
  if (!cameraParamsMap_[IRED1_CAMERA_NAME].gz_topic.empty()) {
    this->transportNode.Subscribe(cameraParamsMap_[IRED1_CAMERA_NAME].gz_topic, &RealSensePlugin::OnNewInfrared1Frame, this);
  }
  if (!cameraParamsMap_[IRED2_CAMERA_NAME].gz_topic.empty()) {
    this->transportNode.Subscribe(cameraParamsMap_[IRED2_CAMERA_NAME].gz_topic, &RealSensePlugin::OnNewInfrared2Frame, this);
  }
}

void RealSensePlugin::OnNewDepthFrame(const gz::msgs::Image & /*_msg*/) {}
void RealSensePlugin::OnNewColorFrame(const gz::msgs::Image & /*_msg*/) {}
void RealSensePlugin::OnNewInfrared1Frame(const gz::msgs::Image & /*_msg*/) {}
void RealSensePlugin::OnNewInfrared2Frame(const gz::msgs::Image & /*_msg*/) {}

}  // namespace realsense_gazebo_plugin
