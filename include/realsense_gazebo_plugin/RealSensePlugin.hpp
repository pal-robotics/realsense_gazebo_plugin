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
#include <vector>
#include <map>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

namespace realsense_gazebo_plugin
{
#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

struct CameraParams
{
  std::string topic_name;
  std::string camera_info_topic_name;
  std::string optical_frame;
  std::string gz_topic;
  double hfov;
};

/// \brief A plugin that simulates Real Sense camera streams via Gazebo Transport.
class RealSensePlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure
{
public:
  RealSensePlugin();
  ~RealSensePlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  virtual void OnNewDepthFrame(const gz::msgs::Image & _msg);
  virtual void OnNewColorFrame(const gz::msgs::Image & _msg);
  virtual void OnNewInfrared1Frame(const gz::msgs::Image & _msg);
  virtual void OnNewInfrared2Frame(const gz::msgs::Image & _msg);

protected:
  std::string prefix;
  gz::transport::Node transportNode;
  std::vector<uint16_t> depthMap;

  std::map<std::string, CameraParams> cameraParamsMap_;

  bool pointCloud_ = false;
  std::string pointCloudTopic_;
  double pointCloudCutOff_, pointCloudCutOffMax_;

  double colorUpdateRate_;
  double infraredUpdateRate_;
  double depthUpdateRate_;

  float rangeMinDepth_;
  float rangeMaxDepth_;
};
}  // namespace realsense_gazebo_plugin