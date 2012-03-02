#include <ros/ros.h>
#include <joy/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


#include "mecanum_kinematic.h"
#include<math.h>

class MecanumOdometryPublisher {
public:
	MecanumOdometryPublisher():mecanum_(220.14, 192.87, 76.175, 100){
		encorder_sub_ = nh_.subscribe<std_msgs::Int32MultiArray> ("encorder",
				50, &MecanumOdometryPublisher::encorderCb, this);
		imu_sub_ = nh_.subscribe<std_msgs::Float32MultiArray> ("imu",
				50, &MecanumOdometryPublisher::imuCb, this);
		odom_pub_ = nh_.advertise<nav_msgs::Odometry> ("odom", 50);

		current_time_ = ros::Time::now();
		last_time_ = ros::Time::now();

		x_=0;
		y_=0;
		th_=0;

		for(int i=0;i<4;i++){
			last_encorder_[i] = 0;
		}
	}

private:
	Mecanum mecanum_;

	ros::NodeHandle nh_;
	ros::Subscriber encorder_sub_,imu_sub_;
	
    ros::Publisher odom_pub_;
	tf::TransformBroadcaster odom_broadcaster_;
	ros::Time current_time_, last_time_;
	
	double x_,y_,th_;
	double imu_[3];
	int last_encorder_[4];
	double vel_imu_th_;

	void imuCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
		imu_[0] = msg->data[0];
		imu_[1] = msg->data[1];
		imu_[2] = msg->data[2];
	}
	
	void encorderCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
		current_time_ = ros::Time::now();
		double dt = (current_time_ - last_time_).toSec();
        
        int vel_encorder[4];
		vel_encorder[0] = last_encorder_[0] - msg->data[0];
		vel_encorder[1] = last_encorder_[1] - msg->data[1];
		vel_encorder[2] = last_encorder_[2] - msg->data[2];
		vel_encorder[3] = last_encorder_[3] - msg->data[3];

		//compute odometry in a typical way given the velocities of the robot
        double vel[3];
		mecanum_.k(&vel_encorder[0],&vel[0]);
		
		//rvizの座標系に合わせて回転
   		double delta_x = -(vel[0] * cos(-th_) - vel[1] * sin(-th_));
		double delta_y = (vel[0] * sin(-th_) + vel[1] * cos(-th_));
		double delta_th = -vel[2];
    	x_ += delta_x;
    	y_ += delta_y;
		th_ += delta_th;
		
    	//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
		//ROS_INFO("x:%f\ty:%f\tth:%f",x_,y_,th_*180/M_PI);

   		//first, we'll publish the transform over tf
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
