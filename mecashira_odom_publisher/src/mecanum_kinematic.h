#ifndef MECANUM_KINEMATIC_H
#define MECANUM_KINEMATIC_H

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
		odom[0] = (omega[0] + omega[1] + omega[2] + omega[3])*m_per_count_/4;
		odom[1] = (omega[0] - omega[1] - omega[2] + omega[3])*m_per_count_/4;
		odom[2] = ((-omega[0]+ omega[1] - omega[2] + omega[3])*mm_per_count_/(4*(l1_+l2_)));

		return odom;
	}

	int* ik(double x,double y,double th)
	{
		int* omega = (int*)malloc(sizeof(int)*4);

		omega[0] = (x * 1000 + y * 1000 - (l1_+l2_) * th)/mm_per_count_;
		omega[1] = (x * 1000 - y * 1000 + (l1_+l2_) * th)/mm_per_count_;
		omega[2] = (x * 1000 - y * 1000 - (l1_+l2_) * th)/mm_per_count_;
		omega[3] = (x * 1000 + y * 1000 + (l1_+l2_) * th)/mm_per_count_;

		return omega;
	}

private:
	double l1_,l2_,r_,mm_per_count_,m_per_count_;
};

#endif
