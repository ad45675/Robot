#include "Header.h"
#include <cmath>

#ifndef KINEMATICS
#define KINEMATICS

#define RAD2DEG(x) x*180/PI
#define DEG2RAD(x) x*PI/180
#define AXIS 6
#define Sol_Num 8



class Kinematics
{
public:

	

	double over180_rad(double input);
	cv::Mat Reverse(cv::Mat nsdt,cv::Mat currentpos);//(double EEx,double EEy,double EEz,cv::Mat nsd,cv::Mat currentpos)
	cv::Mat FindOptimalSol(cv::Mat input,cv::Mat compare);
	cv::Mat FindOptimalSol_YA(cv::Mat input,cv::Mat compare,int Singularplace[Sol_Num],bool &Singular);
	cv::Mat Solution(double Target_Joint[Sol_Num][AXIS],double nowjointpos[AXIS],int Singularplace[Sol_Num],bool &Singular);
	void SetPose(double x,double y,double z,double roll,double pitch,double yall,cv::Mat &output);
	cv::Mat Forward(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6);
	void ForwardJ6(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6 , double* C_NowPos );
	cv::Mat ForwardJ5(double theta1, double theta2, double theta3, double theta4, double theta5);
	cv::Mat ForwardJ4(double theta1, double theta2, double theta3, double theta4);
	cv::Mat ForwardJ3(double theta1, double theta2, double theta3);
	cv::Mat ForwardJ2(double theta1, double theta2);
	cv::Mat ForwardJ1(double theta1);
	cv::Mat Forward_Jnew1(double theta1);
	cv::Mat Forward_Jnew2(double theta1, double theta2);
	cv::Mat Forward_Jnew3(double theta1, double theta2);
	cv::Mat Forward_Jnew4(double theta1, double theta2);
	cv::Mat Forward_Jnew5(double theta1, double theta2, double theta3, double theta4);
	cv::Mat Forward_Jnew6(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6);
	cv::Mat Forward_Jnew7(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6);
	
	void ReverseArray(double nsdt[6],double currentpos[6],double (&output)[6]);

private:
	
	void calculateT(double theta,double alpha,double a,double d, cv::Mat *output);
	void calculateA1(double theta1, cv::Mat *output);
	void calculateA2(double theta1, double theta2, cv::Mat *output);
	void calculateA3(double theta1, double theta2, double theta3, cv::Mat *output);
	void calculateA4(double theta1, double theta2, double theta3, double theta4, cv::Mat *output);
	void calculateA5(double theta1, double theta2, double theta3, double theta4, double theta5, cv::Mat *output);
	void calculateA6(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, cv::Mat *output);
	void calculateJ1(double EEx,double EEy,double EEz,double z1,double z2,double z3,double *J1,double *J1p);
	void calculateJ2J3(double Ex,double Ey,double Ez,double J1,double *J2,double *J2p,double *J3,double *J3p);
	void calculateJ4J5J6(double EEx,double EEy,double EEz,double x1,double x2,double x3,double J1,double J2,double J3,double *J4, double *J5, double *J6,double *J4p, double *J5p, double *J6p);
	
};


#endif // !KINEMATICS