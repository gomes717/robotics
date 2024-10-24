
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/msg/pose.hpp"
using std::placeholders::_1;

class STC: public rclcpp::Node
{
	public:
	STC(void);
	void setControl(const turtlesim::msg::Pose & msg);
			
	private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose target_pose;
};

STC::STC(void): Node("STC")
{
	cmdPublisher_=create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&STC::setControl, this, _1));

    std::srand(std::time(NULL));
    target_pose.x = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;
    target_pose.y = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;

    RCLCPP_INFO_STREAM(get_logger(),"X =" << target_pose.x << " Y=" << target_pose.y);

	// using namespace std::chrono_literals;
	// timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&STC::setCommandCB,this));
}




void STC::setControl(const turtlesim::msg::Pose & msg)
{
	geometry_msgs::msg::Twist cmd;

    // RCLCPP_INFO_STREAM(get_logger(), "Pose received: "
    // << "position=(" <<  msg.x << "," << msg.y << ")"
    // << " direction=" << msg.theta);

    float angle = atan2((target_pose.y - msg.y), (target_pose.x - msg.x));

    
    float angle_diff = (angle - msg.theta);

    float dist = sqrt(pow(msg.y - target_pose.y, 2) + pow(msg.x - target_pose.x, 2));
    RCLCPP_INFO_STREAM(get_logger(),"Angle diff=" << angle_diff << " Dist=" << dist);
    if(angle_diff < -0.05)
    {
        cmd.angular.z=-0.5;
    }
    else if(angle_diff > 0.05)
    {
        cmd.angular.z=0.5;
    }
    else if(dist > 0.1)
    {
        cmd.linear.x = 1.0;
    }
    else
    {
        target_pose.x = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;
        target_pose.y = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;
    }

	cmdPublisher_->publish(cmd);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	
	rclcpp::spin(std::make_shared<STC>());
	
	rclcpp::shutdown();
	return 0;
}
