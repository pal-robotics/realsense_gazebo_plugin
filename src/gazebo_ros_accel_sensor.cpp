/* Copyright [2015] [Alessandro Settimi]
 * 
 * email: ale.settimi@gmail.com
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include "realsense_gazebo_plugin/gazebo_ros_accel_sensor.h"
#include <iostream>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/physics/World.hh>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosAccelSensor)

gazebo::GazeboRosAccelSensor::GazeboRosAccelSensor(): SensorPlugin()
{
  accelerometer_data = ignition::math::Vector3d(0, 0, 0);
  //gyroscope_data = ignition::math::Vector3d(0, 0, 0);
  //orientation = ignition::math::Quaterniond(1,0,0,0);
  seed=0;
  sensor=NULL;
}

void gazebo::GazeboRosAccelSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf=sdf_;
  sensor=dynamic_cast<gazebo::sensors::ImuSensor*>(sensor_.get());

  if(sensor==NULL)
  {
    ROS_FATAL("Error: Sensor pointer is NULL!");
    return;
  }

  sensor->SetActive(true);

  if(!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized()) //check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);

  imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name,1);

  connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosAccelSensor::UpdateChild, this, _1));

  last_time = sensor->LastUpdateTime();
}

void gazebo::GazeboRosAccelSensor::UpdateChild(const gazebo::common::UpdateInfo &/*_info*/)
{
  common::Time current_time = sensor->LastUpdateTime();

  if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) //update rate check
    return;

  if(imu_data_publisher.getNumSubscribers() > 0)
  {
    //orientation = offset.Rot()*sensor->Orientation(); //applying offsets to the orientation measurement
    accelerometer_data = sensor->LinearAcceleration();
    //gyroscope_data = sensor->AngularVelocity();

    //Guassian noise is applied to all measurements
    imu_msg.orientation.x = 0;  //orientation.X() + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.y = 0;  //orientation.Y() + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.z = 0;  //orientation.Z() + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.w = 0;  //orientation.W() + GuassianKernel(0,gaussian_noise);

    imu_msg.linear_acceleration.x = accelerometer_data.X() + GuassianKernel(0,gaussian_noise);
    imu_msg.linear_acceleration.y = accelerometer_data.Y() + GuassianKernel(0,gaussian_noise);
    imu_msg.linear_acceleration.z = accelerometer_data.Z() + GuassianKernel(0,gaussian_noise);

    imu_msg.angular_velocity.x = 0; //gyroscope_data.X() + GuassianKernel(0,gaussian_noise);
    imu_msg.angular_velocity.y = 0; //gyroscope_data.Y() + GuassianKernel(0,gaussian_noise);
    imu_msg.angular_velocity.z = 0; //gyroscope_data.Z() + GuassianKernel(0,gaussian_noise);

    //covariance is related to the Gaussian noise
    double gn2 = gaussian_noise*gaussian_noise;
    imu_msg.orientation_covariance[0] = -1.0; //gn2;
    imu_msg.orientation_covariance[4] = 0;    //gn2;
    imu_msg.orientation_covariance[8] = 0;    //gn2;
    imu_msg.angular_velocity_covariance[0] =    gn2;
    imu_msg.angular_velocity_covariance[4] =    gn2;
    imu_msg.angular_velocity_covariance[8] =    gn2;
    imu_msg.linear_acceleration_covariance[0] = gn2;
    imu_msg.linear_acceleration_covariance[4] = gn2;
    imu_msg.linear_acceleration_covariance[8] = gn2;

    //preparing message header
    imu_msg.header.frame_id = body_name;
    imu_msg.header.stamp.sec = current_time.sec;
    imu_msg.header.stamp.nsec = current_time.nsec;

    //publishing data
    imu_data_publisher.publish(imu_msg);

    ros::spinOnce();
  }

  last_time = current_time;
}

double gazebo::GazeboRosAccelSensor::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool gazebo::GazeboRosAccelSensor::LoadParameters()
{
  //loading parameters from the sdf file

  //NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace =  sdf->Get<std::string>("robotNamespace") +"/";
    ROS_INFO_STREAM("<robotNamespace> set to: "<<robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->ParentName();
    std::size_t it = scoped_name.find("::");

    robot_namespace = "/" +scoped_name.substr(0,it)+"/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
  }

  //TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name =  robot_namespace + sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
  }
  else
  {
    topic_name = robot_namespace + "/accel/sample";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  //BODY NAME
  if (sdf->HasElement("frameName"))
  {
    body_name =  sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("<frameName> set to: "<<body_name);
  }
  else
  {
    ROS_FATAL("missing <frameName>, cannot proceed");
    return false;
  }

  //UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate =  sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
  }

  //NOISE
  if (sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise =  sdf->Get<double>("gaussianNoise");
    ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  }
  else
  {
    gaussian_noise = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  }

  //POSITION OFFSET, UNUSED
  if (sdf->HasElement("xyzOffset"))
  {
    offset.Pos() =  sdf->Get<ignition::math::Vector3d>("xyzOffset");
    ROS_INFO_STREAM("<xyzOffset> set to: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' ' << offset.Pos()[2]);
  }
  else
  {
    offset.Pos() = ignition::math::Vector3d(0, 0, 0);
    ROS_WARN_STREAM("missing <xyzOffset>, set to default: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' ' << offset.Pos()[2]);
  }

  //ORIENTATION OFFSET
  if (sdf->HasElement("rpyOffset"))
  {
    offset.Rot() = ignition::math::Quaterniond(sdf->Get<ignition::math::Vector3d>("rpyOffset"));
    ROS_INFO_STREAM("<rpyOffset> set to: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch() << ' ' << offset.Rot().Yaw());
  }
  else
  {
    offset.Rot() = ignition::math::Quaterniond::Identity;
    ROS_WARN_STREAM("missing <rpyOffset>, set to default: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch() << ' ' << offset.Rot().Yaw());
  }

  return true;
}

gazebo::GazeboRosAccelSensor::~GazeboRosAccelSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}
