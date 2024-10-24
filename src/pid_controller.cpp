
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/msg/pose.hpp"
using std::placeholders::_1;

class PIDC: public rclcpp::Node
{
	public:
	PIDC(void);
	void setControl(const turtlesim::msg::Pose & msg);
			
	private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose target_pose;
    float last_dist;
    float dist_sum;
    float dist_Kp;
    float dist_Ki;
    float dist_Kd;

    float last_angle_err;
    float angle_err_sum;
    float angle_Kp;
    float angle_Ki;
    float angle_Kd;
};

PIDC::PIDC(void): Node("PIDC")
{
	cmdPublisher_=create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&PIDC::setControl, this, _1));

    std::srand(std::time(NULL));
    target_pose.x = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;
    target_pose.y = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;

    RCLCPP_INFO_STREAM(get_logger(),"X =" << target_pose.x << " Y=" << target_pose.y);

    last_dist = 0.0f;
    dist_sum = 0.0f;
    dist_Kp = 0.3f;
    dist_Ki = 0.0f;
    dist_Kd = 0.1f;

    last_angle_err = 0.0f;
    angle_err_sum = 0.0f;
    angle_Kp = 1.0f;
    angle_Ki = 0.00f;
    angle_Kd = 0.0f;

	// using namespace std::chrono_literals;
	// timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&PIDC::setCommandCB,this));
}




void PIDC::setControl(const turtlesim::msg::Pose & msg)
{
	geometry_msgs::msg::Twist cmd;

    // RCLCPP_INFO_STREAM(get_logger(), "Pose received: "
    // << "position=(" <<  msg.x << "," << msg.y << ")"
    // << " direction=" << msg.theta);

    float angle = atan2((target_pose.y - msg.y), (target_pose.x - msg.x));

    
    float angle_diff = (angle - msg.theta);

    float dist = sqrt(pow(msg.y - target_pose.y, 2) + pow(msg.x - target_pose.x, 2));
    RCLCPP_INFO_STREAM(get_logger(),"Angle diff=" << angle_diff << " Dist=" << dist);

    cmd.linear.x = dist_Kp*dist + dist_Kd*(dist - last_dist) + dist_Ki*dist_sum;
    cmd.angular.z = angle_Kp*angle_diff + angle_Kd*(angle_diff - last_angle_err) + angle_Ki*angle_err_sum;
    
    last_dist = dist;
    dist_sum += dist;

    last_angle_err = angle_diff;
    angle_err_sum += angle_diff;


    if(angle_diff > -0.05 && angle_diff < 0.05 && dist < 0.1)
    {
        target_pose.x = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;
        target_pose.y = (static_cast<double>(std::rand()) / RAND_MAX)*11.0;
    }

	cmdPublisher_->publish(cmd);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	
	rclcpp::spin(std::make_shared<PIDC>());
	
	rclcpp::shutdown();
	return 0;
}
