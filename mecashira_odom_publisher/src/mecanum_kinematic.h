#ifndef MECANUM_KINEMATIC_H
#define MECANUM_KINEMATIC_H

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

class Mecanum{
public:
	Mecanum(double l1,double l2,double r,int count)
	{
		l1_ = l2;
		l2_ = l2;
		r_ = r;
		mm_per_count_ = 2*M_PI*r_/(count*4);
		m_per_count_ = mm_per_count_/1000;
	}

	double* k(const int* omega,double*odom)
	{
		//double* odom = (double*)malloc(sizeof(double)*3);

		odom[0] = (omega[0] + omega[1] + omega[2] + omega[3])*m_per_count_/4;
		odom[1] = (omega[0] - omega[1] - omega[2] + omega[3])*m_per_count_/4;
		odom[2] = ((-omega[0]+ omega[1] - omega[2] + omega[3])*mm_per_count_/(4*(l1_+l2_)));

		//pose.position.x = (omega[0] + omega[1] + omega[2] + omega[3])*r_/4;
		//pose.position.y = (omega[0] - omega[1] - omega[2] + omega[3])*r_/4;
		//pose.position.z = 0.0;
		//pose.orientation = tf::createQuaternionMsgFromYaw((-omega[0]+ omega[1] - omega[2] + omega[3])*r_/(4*(l1_+l2_)));

		return odom;
	}

	int* ik(geometry_msgs::Pose pose)
	{
		//int omega[4];
		int* omega = (int*)malloc(sizeof(int)*4);
		double th = tf::getYaw(pose.orientation);

		omega[0] = (pose.position.x*1000 + pose.position.y*1000 - (l1_+l2_) * th)/mm_per_count_;
		omega[1] = (pose.position.x*1000 - pose.position.y*1000 + (l1_+l2_) * th)/mm_per_count_;
		omega[2] = (pose.position.x*1000 - pose.position.y*1000 - (l1_+l2_) * th)/mm_per_count_;
		omega[3] = (pose.position.x*1000 + pose.position.y*1000 + (l1_+l2_) * th)/mm_per_count_;

		//omega[0] = (pose.position.x + pose.position.y - (l1_+l2_) * th)/r_;
		//omega[1] = (pose.position.x - pose.position.y + (l1_+l2_) * th)/r_;
		//omega[2] = (pose.position.x - pose.position.y - (l1_+l2_) * th)/r_;
		//omega[3] = (pose.position.x + pose.position.y + (l1_+l2_) * th)/r_;

		//return &omega[0];
		return omega;
	}


private:
	double l1_,l2_,r_,mm_per_count_,m_per_count_;
};

#endif
