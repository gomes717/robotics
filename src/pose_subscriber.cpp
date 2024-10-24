// This program subscribes to turtle1/pose and shows its
// messages on the screen.

//#include <functional>
//#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class PoseSubscriber : public rclcpp::Node
{
public:
  PoseSubscriber()
  : Node("pose_subscriber")
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&PoseSubscriber::topic_callback, this, _1));
  }

private:
//  void topic_callback(const turtlesim::msg::Pose::ConstPtr& msg)
  void topic_callback(const turtlesim::msg::Pose & msg)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Pose received: "
    << "position=(" <<  msg.x << "," << msg.y << ")"
    << " direction=" << msg.theta);
  }
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
