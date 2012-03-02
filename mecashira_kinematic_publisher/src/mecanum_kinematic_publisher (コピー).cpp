/*
 * mecanum_kinematic_publisher.cpp
 *
 *  Created on: 2011/12/20
 *      Author: javatea
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "mecanum_kinematic.h"
#include <geometry_msgs/Twist.h>
#include<math.h>

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

		current_time_ = ros::Time::now();
		last_time_ = ros::Time::now();

		x_=0;
		y_=0;
		th_=0;

		for(int i=0;i<4;i++){
			last_encorder_[i] = 0;
			vel_encorder_[i] = 0;
			vel_mouse_[i] = 0;
			last_mouse_[i] = 0;
		}

		mouse_x_=0;
		mouse_y_=0;
		mouse_th_=0;
	}

private:
	Mecanum mecanum_;

	ros::NodeHandle nh_;
	ros::Subscriber encorder_sub_,imu_sub_,mouse_sub_;
	ros::Publisher odom_pub_;
	tf::TransformBroadcaster odom_broadcaster_,laser_broadcaster_,mouse_broadcaster_;
	ros::Time current_time_, last_time_;
	double vel_[3];
	double x_,y_,th_;
	double imu_[3];
	int last_mouse_[4],vel_mouse_[4];
	double mouse_x_;
	double mouse_y_;
	double mouse_th_;

	int last_encorder_[4],vel_encorder_[4];

	void imuCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
		imu_[0] = msg->data[0];
		imu_[1] = msg->data[1];
		imu_[2] = msg->data[2];
		//th_ = -imu_[0];//オドメトリ値にIMUを入れるときはアンコメント
		
		//ROS_INFO("%f\t%f\t%f",imu_[0]*180/M_PI,imu_[1]*180/M_PI,imu_[2]*180/M_PI);	
		//ROS_INFO("no imu");
	}
	
	void mouseCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
		
		vel_mouse_[0] = last_mouse_[0] - msg->data[0];
		vel_mouse_[1] = last_mouse_[1] - msg->data[1];
		vel_mouse_[2] = last_mouse_[2] - msg->data[2];
		vel_mouse_[3] = last_mouse_[3] - msg->data[3];

		int vL = -vel_mouse_[1]; //1noY
		int vR = vel_mouse_[3];//2noY
		int vL2 = vel_mouse_[0];//1noX
		int vR2 = -vel_mouse_[2];//2noX

        double omega = (vR - vL)/(139.0);
        double v = (vR + vL)/2.0;

        double omega2 = (vR2 - vL2)/(75.0);
        double v2 = (vR2 + vL2)/2.0;

		mouse_th_ += (omega+omega2)/2.0*25.4/400.0;
		double delta_x = v*cos(-mouse_th_) + v2*sin(-mouse_th_)*25.4/400.0;
		double delta_y = v*sin(-mouse_th_) - v2*cos(-mouse_th_)*25.4/400.0;        

		mouse_x_ += delta_x;
		mouse_y_ += delta_y;

		//ROS_INFO("mouse cb");
		ROS_INFO("mx:%f\tmy:%f\tm:th%f",mouse_y_/1000.0,mouse_x_/1000.0,mouse_th_ *180/M_PI);
		//ROS_INFO("%d %d %d %d",vR,vL,vR2,vL2);

		geometry_msgs::TransformStamped mouse_trans;
		mouse_trans.header.stamp = current_time_;
		mouse_trans.header.frame_id = "odom";
		mouse_trans.child_frame_id = "mouse";
    	mouse_trans.transform.translation.x = mouse_y_/1000.0;
    	mouse_trans.transform.translation.y = mouse_x_/1000.0;
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

		vel_encorder_[0] = last_encorder_[0] - msg->data[0];
		vel_encorder_[1] = last_encorder_[1] - msg->data[1];
		vel_encorder_[2] = last_encorder_[2] - msg->data[2];
		vel_encorder_[3] = last_encorder_[3] - msg->data[3];


		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time_ - last_time_).toSec();
		mecanum_.k(&vel_encorder_[0],&vel_[0]);;
   		double delta_x = -(vel_[0] * cos(-th_) - vel_[1] * sin(-th_)); //* dt;
		double delta_y = (vel_[0] * sin(-th_) + vel_[1] * cos(-th_)); //* dt;
		double delta_th = -vel_[2];// * dt;

  		x_ += delta_x;
    	y_ += delta_y;
    	th_ += delta_th;//オドメトリ値にIMUを入れてるときはコメントアウト
		
    	//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
		
		//ROS_INFO("vx:%f\tvy:%f\tvth:%f",vel_[0],vel_[1],vel_[2]*180/M_PI);
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

		tf::StampedTransform transform;
    	transform.setOrigin( tf::Vector3(0.12, 0.0, 0.0) );
   		transform.setRotation( tf::Quaternion(0, 0, 0) );
		laser_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));

/*
    	geometry_msgs::TransformStamped laser_trans;
    	laser_trans.header.stamp = current_time_;
    	laser_trans.header.frame_id = "odom";
    	laser_trans.child_frame_id = "laser";
    	laser_trans.transform.translation.x = x_ - delta_x  + ((vel_[0]+0.12) * cos(th_) - vel_[1] * sin(th_));
    	laser_trans.transform.translation.y = y_ - delta_y  + ((vel_[0]+0.12) * sin(th_) + vel_[1] * cos(th_));
    	laser_trans.transform.translation.z = 0.0;
    	laser_trans.transform.rotation = odom_quat;
*/
  	  	//send the transform
    	odom_broadcaster_.sendTransform(odom_trans);
		//laser_broadcaster_.sendTransform(laser_trans);

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
