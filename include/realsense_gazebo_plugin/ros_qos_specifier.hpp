#include "realsense_gazebo_plugin/RealSensePlugin.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <string>

#ifndef REALSENSE_GAZEBO_PLUGIN_ROS_QOS_SPECIFIER_HPP
#define REALSENSE_GAZEBO_PLUGIN_ROS_QOS_SPECIFIER_HPP

class ImageTransportWithSpecifiedQos: public image_transport::ImageTransport {
public:
    explicit ImageTransportWithSpecifiedQos(rclcpp::Node::SharedPtr node) : ImageTransport(node),
    queue_size(2),ad_node_p(node){};
    image_transport::CameraPublisher advertise_camera(
            const std::string & base_topic, uint32_t queue_size, rmw_qos_profile_t qos_profile);

    void specify_color_qos(
            image_transport::CameraPublisher &cam_color_pub, std::basic_string<char> topic_name, std::string colorQos);

private:
    rclcpp::Node::SharedPtr ad_node_p;
    uint32_t queue_size;
};

#endif //REALSENSE_GAZEBO_PLUGIN_ROS_QOS_SPECIFIER_HPP


