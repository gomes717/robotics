/******************************************************************************
	                   ROS2 Pubilhser Example
          Copyright (C) 2021 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Geneal Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.

*******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TurtleCmd: public rclcpp::Node
{
	public:
	TurtleCmd(void);
	void setCommandCB(void) const;
			
	private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

TurtleCmd::TurtleCmd(void): Node("turtle_command")
{
	cmdPublisher_=create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

	using namespace std::chrono_literals;
	timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&TurtleCmd::setCommandCB,this));
}

void TurtleCmd::setCommandCB(void) const
{
	geometry_msgs::msg::Twist cmd;
	
	cmd.linear.x=1.0;
	cmd.linear.y=1.0;
	cmd.linear.z=0.0;

	cmd.angular.x=0.0;
	cmd.angular.y=0.0;
	cmd.angular.z=1.0;

	cmdPublisher_->publish(cmd);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	
	rclcpp::spin(std::make_shared<TurtleCmd>());
	
	rclcpp::shutdown();
	return 0;
}
