#include <ros/ros.h>
#include <joy/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


#include "mecanum_kinematic.h"
#include<math.h>

#define STOP    0x00
#define FORWARD 0x01
#define BACK    0x05
#define RIGHT   0x03
#define LEFT    0x07
#define ROTATE_LEFT	0x0A
#define ROTATE_RIGHT 0x09

class MecanumOdometryPublisher {
public:
	MecanumOdometryPublisher():mecanum_(220.14, 192.87, 76.175, 100){
		encorder_sub_ = nh_.subscribe<std_msgs::Int32MultiArray> ("encorder",
				50, &MecanumOdometryPublisher::encorderCb, this);
		imu_sub_ = nh_.subscribe<std_msgs::Float32MultiArray> ("imu",
				50, &MecanumOdometryPublisher::imuCb, this);
		mouse_sub_ = nh_.subscribe<std_msgs::Int32MultiArray> ("mouse",
				50, &MecanumOdometryPublisher::mouseCb, this);

		odom_pub_ = nh_.advertise<nav_msgs::Odometry> ("odom", 50);

        joy_sub_ = nh_.subscribe<joy::Joy> ("joy", 10,&MecanumOdometryPublisher::joyCallback, this);

		current_time_ = ros::Time::now();
		last_time_ = ros::Time::now();

		x_=0;
		y_=0;
		th_=0;

		for(int i=0;i<4;i++){
			last_encorder_[i] = 0;
			last_mouse_[i] = 0;
		}

		mouse_x_=0;
		mouse_y_=0;
		mouse_th_=0;
		last_imu_th_=0;
		vel_imu_th_=0;
        ctrl_ = STOP;
        MOUSE_BIAS = 25.4/800;
	}

private:
	Mecanum mecanum_;

	ros::NodeHandle nh_;
	ros::Subscriber encorder_sub_,imu_sub_,mouse_sub_;
    ros::Subscriber joy_sub_;
	
    ros::Publisher odom_pub_;
	tf::TransformBroadcaster odom_broadcaster_,mouse_broadcaster_;
	tf::TransformListener listener_;
	ros::Time current_time_, last_time_;

	double x_,y_,th_;
	double imu_[3];
	int last_mouse_[4],last_encorder_[4];
	double mouse_x_,mouse_y_,mouse_th_;
	double vel_imu_th_,last_imu_th_;
	double vel_mouse_th_;

    unsigned char ctrl_;
    double MOUSE_BIAS;

	void joyCallback(const joy::Joy::ConstPtr& joy) {
		//back
		if (joy->axes[0] == 0 && joy->axes[1] == -1 && joy->buttons[6] == 0&& joy->buttons[7] == 0)  {
			ROS_INFO("back");
			ctrl_ = BACK;
		}//forward
		else if (joy->axes[0] == 0 && joy->axes[1] == 1 && joy->buttons[6] == 0&& joy->buttons[7] == 0){
			ROS_INFO("forward");
			ctrl_ = FORWARD;
		}//left
		else if (joy->axes[0] == 1 && joy->axes[1] == 0 && joy->buttons[6] == 0&& joy->buttons[7] == 0){
			ROS_INFO("left");
			ctrl_ = LEFT;
		}//right
		else if (joy->axes[0] == -1 && joy->axes[1] == 0 && joy->buttons[6] == 0&& joy->buttons[7] == 0){
			ROS_INFO("right");
			ctrl_ = RIGHT;
		}
		else if(joy->axes[0] == 0 && joy->axes[1] == 0 && joy->buttons[6] == 1&& joy->buttons[7] == 0){
			ROS_INFO("roate left");
			ctrl_ = ROTATE_LEFT;
		}
		else if(joy->axes[0] == 0 && joy->axes[1] == 0 && joy->buttons[6] == 0&& joy->buttons[7] == 1){
			ROS_INFO("rotate right");
			ctrl_ = ROTATE_RIGHT;
		}
		else{
			ROS_INFO("stop");
			ctrl_ = STOP;
		}
	}

	void imuCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
		if(ctrl_ == ROTATE_LEFT || ctrl_ == ROTATE_RIGHT)
		{
			vel_imu_th_ = last_imu_th_ - msg->data[0];
		}

		imu_[0] = msg->data[0];
		imu_[1] = msg->data[1];
		imu_[2] = msg->data[2];
		//th_ = -imu_[0];//オドメトリ値にIMUを入れるときはアンコメント
		
		//ROS_INFO("%f\t%f\t%f",imu_[0]*180/M_PI,imu_[1]*180/M_PI,imu_[2]*180/M_PI);	
		//ROS_INFO("no imu");
		last_imu_th_=msg->data[0];
	}
	
	void mouseCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        int vel_mouse[4] = {0,0,0,0};

        if(ctrl_ != STOP){
		    vel_mouse[0] = last_mouse_[0] - msg->data[0];
		    vel_mouse[1] = last_mouse_[1] - msg->data[1];
		    vel_mouse[2] = last_mouse_[2] - msg->data[2];
		    vel_mouse[3] = last_mouse_[3] - msg->data[3];
        }
		int vL = -vel_mouse[1]; //1noY
		int vR = vel_mouse[3];//2noY

		int vL2 = vel_mouse[0];//1noX
		int vR2 = -vel_mouse[2];//2noX 0.996

        double omega = (vR - vL)/(139.0);
        double v = (vR + vL)/2.0;

        double omega2 = (vR2 - vL2)/(75.0);
        double v2 = (vR2 + vL2)/2.0;

		vel_mouse_th_ = (-omega+omega2)/2.0*MOUSE_BIAS;

		if(vel_mouse_th_<0){
			vel_mouse_th_*=0.97;//0.56 0.49
		}else{
			vel_mouse_th_*=0.77;
		}

		mouse_th_ += vel_mouse_th_;

		double delta_x = (v*cos(-mouse_th_) + v2*sin(-mouse_th_))*MOUSE_BIAS;
		double delta_y = (v*sin(-mouse_th_) - v2*cos(-mouse_th_))*MOUSE_BIAS;        
		if(delta_y > 0)
		{
			//delta_y*=0.85;
		}

		mouse_x_ += delta_x;
		mouse_y_ += delta_y;

		ROS_INFO("mx:%f\tmy:%f\tm:th%f",mouse_y_/1000.0,mouse_x_/1000.0,mouse_th_ *180/M_PI);
		//ROS_INFO("%d %d %d %d",vR,vL,vR2,vL2);

		tf::StampedTransform transform;
		listener_.lookupTransform("/base_link2", "/mouse",  ros::Time(0), transform);
		

		geometry_msgs::TransformStamped mouse_trans;
		mouse_trans.header.stamp = current_time_;
		mouse_trans.header.frame_id = "odom";
		mouse_trans.child_frame_id = "mouse";
    	mouse_trans.transform.translation.x = mouse_y_/1000.0;
    	mouse_trans.transform.translation.y = mouse_x_/1000.0 + transform.getOrigin().y();
		mouse_trans.transform.translation.z = 0.0;
		mouse_trans.transform.rotation = tf::createQuaternionMsgFromYaw(mouse_th_);		

		mouse_broadcaster_.sendTransform(mouse_trans);


		last_mouse_[0] = msg->data[0];
		last_mouse_[1] = msg->data[1];
		last_mouse_[2] = msg->data[2];
		last_mouse_[3] = msg->data[3];
	}


	void encorderCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
		current_time_ = ros::Time::now();
        int vel_encorder[4] = {0,0,0,0};
        double vel[3];

		vel_encorder[0] = last_encorder_[0] - msg->data[0];
		vel_encorder[1] = last_encorder_[1] - msg->data[1];
		vel_encorder[2] = last_encorder_[2] - msg->data[2];
		vel_encorder[3] = last_encorder_[3] - msg->data[3];


		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time_ - last_time_).toSec();
		mecanum_.k(&vel_encorder[0],&vel[0]);
   		double delta_x = -(vel[0] * cos(-th_) - vel[1] * sin(-th_));
		double delta_y = (vel[0] * sin(-th_) + vel[1] * cos(-th_));
		double delta_th = -vel[2];

  		x_ += delta_x;
    	y_ += delta_y;
		if(ctrl_ == ROTATE_LEFT || ctrl_ == ROTATE_RIGHT){
			th_ += vel_mouse_th_;//回転中はマウスのデータを使ってみる
    		//th_ += vel_imu_th_;//回転中はIMUのデータを使ってみる
			//th_ +=delta_th;
		}else{
			
			th_ += delta_th;
		}
    	//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
		
		ROS_INFO("x:%f\ty:%f\tth:%f",x_,y_,th_*180/M_PI);

   		//first, we'll publish the transform over tf
		//*****Odometry*****//
    	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = current_time_;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_link";
    	odom_trans.transform.translation.x = x_;
    	odom_trans.transform.translation.y = y_;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = odom_quat;

  	  	//send the transform
    	odom_broadcaster_.sendTransform(odom_trans);

    	//next, we'll publish the odometry message over ROS
    	nav_msgs::Odometry odom;
    	odom.header.stamp = current_time_;
    	odom.header.frame_id = "odom";
    	//set the position
    	odom.pose.pose.position.x = x_;
    	odom.pose.pose.position.y = y_;
    	odom.pose.pose.position.z = 0.0;
    	odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
    	odom.child_frame_id = "base_link";
    	odom.twist.twist.linear.x = delta_x/dt;
    	odom.twist.twist.linear.y = delta_y/dt;
   	 	odom.twist.twist.angular.z = delta_th/dt;

		//publish the message
		odom_pub_.publish(odom);

		last_time_ = ros::Time::now();

		last_encorder_[0] = msg->data[0];
		last_encorder_[1] = msg->data[1];
		last_encorder_[2] = msg->data[2];
		last_encorder_[3] = msg->data[3];
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mecanum_odomery_publisher");
	MecanumOdometryPublisher mecanum_odometry_publisher;

	ros::spin();
}
