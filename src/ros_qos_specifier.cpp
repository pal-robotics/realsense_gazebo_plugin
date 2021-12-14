#include "realsense_gazebo_plugin/ros_qos_specifier.hpp"

image_transport::CameraPublisher gazebo::ImageTransportWithSpecifiedQos::advertise_camera(
        const std::string & base_topic, uint32_t queue_size, rmw_qos_profile_t qos_profile)
{
    qos_profile.depth = queue_size;
    return image_transport::CameraPublisher(ad_node_p.get(), base_topic, qos_profile);
}

void gazebo::ImageTransportWithSpecifiedQos::specify_color_qos(
        image_transport::CameraPublisher &cam_color_pub,
        std::basic_string<char> topic_name, std::string colorQos)
{
  if(colorQos=="SensorDataQoS") {
    cam_color_pub = this->advertise_camera(topic_name, 2,rclcpp::SensorDataQoS().get_rmw_qos_profile());
    RCLCPP_INFO(this->ad_node_p->get_logger(), "Gazebo ROS Realsense plugin -> advertise camera using SensorDataQoS.");
  } else if(colorQos=="ParametersQoS") {
    cam_color_pub = this->advertise_camera(
            topic_name, 2,rclcpp::ParametersQoS().get_rmw_qos_profile());
    RCLCPP_INFO(this->ad_node_p->get_logger(), "Gazebo ROS Realsense plugin -> advertise camera using ParametersQoS.");
  } else if(colorQos=="ServicesQoS") {
    cam_color_pub = this->advertise_camera(
            topic_name, 2,rclcpp::ServicesQoS().get_rmw_qos_profile());
    RCLCPP_INFO(this->ad_node_p->get_logger(), "Gazebo ROS Realsense plugin -> advertise camera using ServicesQoS.");
  } else if(colorQos=="ParameterEventsQoS") {
    cam_color_pub = this->advertise_camera(
            topic_name, 2,rclcpp::ParameterEventsQoS().get_rmw_qos_profile());
    RCLCPP_INFO(this->ad_node_p->get_logger(), "Gazebo ROS Realsense plugin -> advertise camera using ParameterEventsQoS.");
  } else  {
    cam_color_pub = this->advertise_camera(
            topic_name, 2,rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());
    RCLCPP_INFO(this->ad_node_p->get_logger(), "Gazebo ROS Realsense plugin -> advertise camera using SystemDefaultsQoS.");
  }
}