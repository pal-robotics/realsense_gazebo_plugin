#include <gtest/gtest.h>

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


constexpr char COLOR_TOPIC[] = "/realsense/camera/color/image_raw";
constexpr char IR1_TOPIC[]   = "/realsense/camera/ir/image_raw";
constexpr char IR2_TOPIC[]   = "/realsense/camera/ir2/image_raw";
constexpr char DEPTH_TOPIC[] = "/realsense/camera/depth/image_raw";

// See sensor_msgs::image_encodings
constexpr char OPENCV_RGB[]    = "8UC3";
constexpr char OPENCV_MONO[]   = "8UC1";
constexpr char OPENCV_MONO16[] = "16UC1";


template <const char *topic, const char *opencv_pixel_format>
class ImageStreamTest : public ::testing::Test
{
public:
  bool msg_received;
  uint32_t count;
  std::string encoding_recv;
  int width_recv;
  int height_recv;
  int step_recv;
  int caminfo_height_recv;
  int caminfo_width_recv;
  float average;
  float caminfo_D_recv[5];

public:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;

public:
  ImageStreamTest() :
    nh(),
    it(nh),
    sub(it.subscribe(topic, 1, &ImageStreamTest::Callback, this))
  {
    msg_received = false;
    count = 0;
  }

  // Necessary because it takes a while for the node under test to start up
  void SetUp()
  {
    while (!(sub.getNumPublishers() > 0))
    {
      ros::spinOnce();
    }
  }

  // It takes time for messages from the node under test to reach this node.
  void WaitForMessage()
  {
    while(!msg_received)
    {
      ros::spinOnce();
    }
  }

public:
  void storeMsgInfo(const sensor_msgs::ImageConstPtr &msg)
  {
    encoding_recv = msg->encoding;
    width_recv = msg->width;
    height_recv = msg->height;
    step_recv = msg->step;
  }

  void storeCameraInfo(const sensor_msgs::CameraInfoConstPtr &info_msg)
  {
    caminfo_height_recv = info_msg->height;
    caminfo_width_recv = info_msg->width;
  }

  void Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    count++;
    msg_received = true;

    cv::Mat image = cv_bridge::toCvShare(msg, opencv_pixel_format)->image;

    uchar *data = image.data;

    long total_intensity = 0;
    int nb_pixels = 1;
    for (unsigned int i = 0; i < msg->height * msg->width; i++)
    {
      if (*data > 0 && *data < 255)
      {
        total_intensity += *data;
        nb_pixels++;
      }
      data++;
    }
    if (nb_pixels != 0)
    {
      average = total_intensity / nb_pixels;
    }

    storeMsgInfo(msg);
  }
};


using ColorStreamTest = ImageStreamTest<COLOR_TOPIC, OPENCV_RGB>;

TEST_F(ColorStreamTest, testStream)
{
  this->WaitForMessage();
  EXPECT_TRUE(this->count > 0);
  EXPECT_TRUE(this->average > 0);
}

TEST_F(ColorStreamTest, testResolution)
{
  this->WaitForMessage();
  EXPECT_EQ(640, width_recv);
  EXPECT_EQ(480, height_recv);
}

TEST_F(ColorStreamTest, testCameraInfo)
{
  this->WaitForMessage();
  EXPECT_EQ("rgb8", encoding_recv);
  EXPECT_EQ(640 * 3, step_recv);
}


using Ired1StreamTest = ImageStreamTest<IR1_TOPIC, OPENCV_MONO>;

TEST_F(Ired1StreamTest, testStream)
{
  this->WaitForMessage();
  EXPECT_TRUE(this->count > 0);
  EXPECT_TRUE(this->average > 0);
}

TEST_F(Ired1StreamTest, testResolution)
{
  this->WaitForMessage();
  EXPECT_EQ(640, width_recv);
  EXPECT_EQ(480, height_recv);
}

TEST_F(Ired1StreamTest, testCameraInfo)
{
  this->WaitForMessage();
  EXPECT_EQ("mono8", encoding_recv);
  EXPECT_EQ(640 * 1, step_recv);
}


using Ired2StreamTest = ImageStreamTest<IR2_TOPIC, OPENCV_MONO>;

TEST_F(Ired2StreamTest, testStream)
{
  this->WaitForMessage();
  EXPECT_TRUE(this->count > 0);
  EXPECT_TRUE(this->average > 0);
}

TEST_F(Ired2StreamTest, testResolution)
{
  this->WaitForMessage();
  EXPECT_EQ(640, width_recv);
  EXPECT_EQ(480, height_recv);
}

TEST_F(Ired2StreamTest, testCameraInfo)
{
  this->WaitForMessage();
  EXPECT_EQ("mono8", encoding_recv);
  EXPECT_EQ(640 * 1, step_recv);
}


using DepthStreamTest = ImageStreamTest<DEPTH_TOPIC, OPENCV_MONO16>;

TEST_F(DepthStreamTest, testStream)
{
  this->WaitForMessage();
  EXPECT_TRUE(this->count > 0);
  EXPECT_TRUE(this->average > 0);
}

TEST_F(DepthStreamTest, testResolution)
{
  this->WaitForMessage();
  EXPECT_EQ(640, width_recv);
  EXPECT_EQ(480, height_recv);
}

TEST_F(DepthStreamTest, testCameraInfo)
{
  this->WaitForMessage();
  EXPECT_EQ("mono16", encoding_recv);
  EXPECT_EQ(640 * 2, step_recv);
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "realsense-plugin-unittest");

  ROS_INFO_STREAM("RealSense Gazebo ROS plugin - Starting Tests...");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return ret;
}
