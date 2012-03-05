#include <ros/ros.h>
#include <joy/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

#include "mecanum_kinematic.h"
#include<math.h>

#define ROBOT_LENGTH1   220.14
#define ROBOT_LENGTH2   192.87
#define WHEEL_RADIUS    76.175
#define ENCORDER_COUNT  100

class MecanumOdometryPublisher {
public:
	MecanumOdometryPublisher():mecanum_(ROBOT_LENGTH1, ROBOT_LENGTH2, WHEEL_RADIUS, ENCORDER_COUNT){
		encorder_sub_ = nh_.subscribe<std_msgs::Int32MultiArray> ("encorder",
			50, &MecanumOdometryPublisher::encorderCb, this);
		imu_sub_ = nh_.subscribe<std_msgs::Float32MultiArray> ("imu",
			50, &MecanumOdometryPublisher::imuCb, this);
		robot_goal_sub_ = nh_.subscribe<std_msgs::Float32MultiArray> ("robot_goal",
        	50, &MecanumOdometryPublisher::robot_goalCb, this);
		sensor_reset_sub_ = nh_.subscribe<std_msgs::Empty> ("sensor_reset_sub",
        	10, &MecanumOdometryPublisher::sensor_resetCb, this);

		odom_pub_ = nh_.advertise<nav_msgs::Odometry> ("odom", 50);
		motor_goal_pub_ = nh_.advertise<std_msgs::Int32MultiArray> ("motor_goal", 50);

		current_time_ = ros::Time::now();
		last_time_ = ros::Time::now();

		x_=0;
	    y_=0;
	    th_=0;
		for(int i=0;i<4;i++){
    	    last_encorder_[i] = 0;
    	}
    	
    	ros::NodeHandle n_param ("~");
    	if (!n_param.getParam("odom_angular_scale_correction", odom_angular_scale_correction_)){
			odom_angular_scale_correction_ = 1.0;
		}
	}

private:
	Mecanum mecanum_;

	ros::NodeHandle nh_;

	ros::Subscriber encorder_sub_,imu_sub_,robot_goal_sub_,sensor_reset_sub_;
    ros::Publisher odom_pub_,motor_goal_pub_;

	tf::TransformBroadcaster odom_broadcaster_;
	ros::Time current_time_, last_time_;
	
	double x_,y_,th_;
	double imu_[3];
	int last_encorder_[4];
	double vel_imu_th_;

	double odom_angular_scale_correction_;

	void sensor_resetCb(const std_msgs::Empty::ConstPtr& msg){
		x_=0;
		y_=0;
		th_=0;
		for(int i=0;i<4;i++){
			last_encorder_[i] = 0;
		}
	}

	void robot_goalCb(const std_msgs::Float32MultiArray::ConstPtr& msg){
		std_msgs::Int32MultiArray motorgoal_msg;
		motorgoal_msg.data.clear();
		
		int motorgoal_count[4];
		mecanum_.ik(msg->data[0],-msg->data[1],msg->data[2], &motorgoal_count[0]);
		//ROS_INFO("%d %d %d %d",motorgoal_count[0],motorgoal_count[1],motorgoal_count[2],motorgoal_count[3]);
		
		for(int i=0; i<4; i++){
			motorgoal_msg.data.push_back(motorgoal_count[i] / odom_angular_scale_correction_ + last_encorder_[i]);
		}
		
		motor_goal_pub_.publish(motorgoal_msg);
	}
	
	void encorderCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
		current_time_ = ros::Time::now();
		double dt = (current_time_ - last_time_).toSec();
        
        int vel_encorder[4];
		for(int i=0; i<4; i++){
			vel_encorder[i] = msg->data[i] - last_encorder_[i];
		}

		//compute odometry in a typical way given the velocities of the robot
        double vel[3];
		mecanum_.k(&vel_encorder[0],&vel[0]);
		
		//rvizの座標系に合わせて回転したり
   		double delta_x = (vel[0] * cos(th_) - vel[1] * sin(th_));
		double delta_y = -(vel[0] * sin(th_) + vel[1] * cos(th_));
		double delta_th = vel[2] * odom_angular_scale_correction_;
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
		for(int i=0; i<4; i++){
			last_encorder_[i] = msg->data[i];
		}
	}

	void imuCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
		imu_[0] = msg->data[0];
		imu_[1] = msg->data[1];
		imu_[2] = msg->data[2];
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mecashira_odomery_publisher");
	MecanumOdometryPublisher mecanum_odometry_publisher;

	ros::spin();
}
