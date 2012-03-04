#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h>
#include <joy/Joy.h>

//ファームウェアへの通信コマンド
#define STOP    0x00
#define FORWARD 0x01
#define BACK    0x05
#define RIGHT   0x03
#define LEFT    0x07
#define ROTATE_LEFT	0x0A
#define ROTATE_RIGHT 0x09


class TeleopRobot {
public:
	TeleopRobot() {
		vel_pub_ = nh_.advertise<std_msgs::UInt8> ("robot_speed", 1);
		vel_msg.data = 20;
		ctrl_pub_ = nh_.advertise<std_msgs::UInt8> ("joy_ctrl", 1);
		joy_sub_ = nh_.subscribe<joy::Joy> ("joy", 10,&TeleopRobot::joyCallback, this);
		
		//parameter
		ros::NodeHandle n_param ("~");
    	if (!n_param.getParam("speed_up_button", speed_up_button_)){
			speed_up_button_ = 3;
		}
    	if (!n_param.getParam("speed_down_button", speed_down_button_)){
			speed_down_button_ = 1;
		}
    	if (!n_param.getParam("rotate_left_button", rotate_left_button_)){
			rotate_left_button_ = 6;
		}
    	if (!n_param.getParam("rotate_right_button", rotate_right_button_)){
			rotate_right_button_ = 7;
		}
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Publisher ctrl_pub_;
	std_msgs::UInt8 vel_msg;
	ros::Subscriber joy_sub_;
	std_msgs::UInt8 ctrl_msg;

	int speed_up_button_,speed_down_button_,rotate_left_button_,rotate_right_button_;
	

	void joyCallback(const joy::Joy::ConstPtr& joy) {
		ROS_INFO("joystick input");

		if (joy->buttons[speed_up_button_ ] == 1 && joy->buttons[speed_down_button_] == 0 && vel_msg.data < 28)
		{
			vel_msg.data += 2;
			vel_pub_.publish(vel_msg);
		}
		else if (joy->buttons[speed_up_button_ ] == 0 && joy->buttons[speed_down_button_] == 1 && vel_msg.data > 0)
		{
			vel_msg.data -= 2;
			vel_pub_.publish(vel_msg);
		}


		ctrl_msg.data = STOP;
		//back
		if (joy->axes[0] == 0 && joy->axes[1] == -1)  {
			ctrl_msg.data = BACK;
		}//forward
		else if (joy->axes[0] == 0 && joy->axes[1] == 1){
			ctrl_msg.data = FORWARD;
		}//left
		else if (joy->axes[0] == 1 && joy->axes[1] == 0){
			ctrl_msg.data = LEFT;
		}//right
		else if (joy->axes[0] == -1 && joy->axes[1] == 0){
			ctrl_msg.data = RIGHT;
		}

		if(!ctrl_msg.data){
			//l1 rotate left
			if( joy->buttons[rotate_left_button_] == 1&& joy->buttons[rotate_right_button_] == 0){
				ctrl_msg.data = ROTATE_LEFT;
			}//r1 rotate right
			else if(joy->buttons[rotate_left_button_] == 0 && joy->buttons[rotate_right_button_] == 1){
				ctrl_msg.data = ROTATE_RIGHT;
			}
		}
		ctrl_pub_.publish(ctrl_msg);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_mecashira");
	TeleopRobot teleop_motor;

	ros::spin();
}
