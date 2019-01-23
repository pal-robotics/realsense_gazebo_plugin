#include "myproject/mynode.h"

#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

// TODO: add other includes as needed

namespace myproject {
class MyNodeTest : public ::testing::Test {
 public:
  MyNodeTest()
      : node_handle_(),
        publisher_(
            node_handle_.advertise<Feedback>(
                "/input_topic", 5)),
        subscriber_(
            node_handle_.subscribe("/output_topic", 5,
                                   &MyNodeTest::Callback,
                                   this)),
        message_received_(false) {
  }

  /*
   * This is necessary because it takes a while for the node under
   * test to start up.
   */
  void SetUp() {
    while (!IsNodeReady()) {
      ros::spinOnce();
    }
  }

  void Publish(int num /* TODO: add necessary fields */) {
    MyInputMessage message;
    // TODO: construct message.
    publisher_.publish(message);
  }

  /*
   * This is necessary because it takes time for messages from the
   * node under test to reach this node.
   */
  boost::shared_ptr<const MyOutputMessage> WaitForMessage() {
    // The second parameter is a timeout duration.
    return ros::topic::waitForMessage<MyOutputMessage>(
        subscriber_.getTopic(), ros::Duration(1));
  }

  // An alternative way of waiting for a message.
  // ros::topic::waitForMessage can sometimes be flaky.
  void WaitForMessageAlternate() {
    while(!message_received_) {
      ros::spinOnce();
    }
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  bool message_received_;

  /*
   * This callback is a no-op because we get the messages from the
   * node under test using WaitForMessage().
   */
  void Callback(const MyOutputMessage& event) {
    message_received_ = true;
  }

  /*
   * See SetUp method.
   */
  bool IsNodeReady() {
    return (publisher_.getNumSubscribers() > 0)
        && (subscriber_.getNumPublishers() > 0);
  }
};

TEST_F(MyNodeTest, NonZeroInputDoesSomething) {
  Publish(1);
  auto output = WaitForMessage();
  ASSERT_TRUE(output != NULL);
  EXPECT_EQ(10, output->some_output);
}

TEST_F(MyNodeTest, ZeroInputDoesNothing) {
  Publish(0);
  auto output = WaitForMessage();
  ASSERT_TRUE(output == NULL);
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mynode_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
