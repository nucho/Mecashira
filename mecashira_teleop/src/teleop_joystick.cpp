/*
 * joy_motor.cpp
 *
 *  Created on: 2011/10/21
 *      Author: javatea
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h>
#include <joy/Joy.h>

class TeleopRobot {
public:
	TeleopRobot() {
		vel_pub_ = nh_.advertise<std_msgs::UInt8> ("robot_speed", 1);
		vel_msg.data = 24;
		//goal_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("motor_goal", 1);
		ctrl_pub_ = nh_.advertise<std_msgs::UInt8> ("joy_ctrl", 1);
		joy_sub_ = nh_.subscribe<joy::Joy> ("joy", 10,&TeleopRobot::joyCallback, this);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
//	ros::Publisher goal_pub_;
	ros::Publisher ctrl_pub_;
	std_msgs::UInt8 vel_msg;
	ros::Subscriber joy_sub_;
	std_msgs::UInt8 ctrl_msg;

	void joyCallback(const joy::Joy::ConstPtr& joy) {
		ROS_INFO("joystick input");

		if (joy->buttons[0] == 1 && joy->buttons[2] == 0 && vel_msg.data < 48)
		{
			vel_msg.data += 4;
			vel_pub_.publish(vel_msg);
		}
		else if (joy->buttons[0] == 0 && joy->buttons[2] == 1 && vel_msg.data > 0)
		{
			vel_msg.data -= 4;
			vel_pub_.publish(vel_msg);
		}

		ctrl_msg.data = 0;
		//back
		if (joy->axes[0] == 0 && joy->axes[1] == -1)  {
			ctrl_msg.data = 0x05;
		}//forward
		else if (joy->axes[0] == 0 && joy->axes[1] == 1){
			ctrl_msg.data = 0x01;
		}//left
		else if (joy->axes[0] == 1 && joy->axes[1] == 0){
			ctrl_msg.data = 0x07;
		}//right
		else if (joy->axes[0] == -1 && joy->axes[1] == 0){
			ctrl_msg.data = 0x03;
		}

		if(!ctrl_msg.data){
			//l1 rotate left
			if( joy->buttons[6] == 1&& joy->buttons[7] == 0){
				ctrl_msg.data = 0x0A;
			}//r1 rotate right
			else if(joy->buttons[6] == 0 && joy->buttons[7] == 1){
				ctrl_msg.data = 0x09;
			}
		}
		ctrl_pub_.publish(ctrl_msg);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_robot");
	TeleopRobot teleop_motor;

	ros::spin();
}
