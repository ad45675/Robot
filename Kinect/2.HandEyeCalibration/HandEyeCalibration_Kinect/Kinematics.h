#ifndef KINEMATICS
#define KINEMATICS


#ifndef OPENCV_HPP
#define OPENCV_HPP 

#include <opencv2/opencv.hpp> 
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#endif

#ifndef PARAMETER
#define PARAMETER 

#define PI 3.14159265358979323846
#define RAD2DEG(x) x*180/PI
#define DEG2RAD(x) x*PI/180

#endif

class Kinematics
{
public:

	double over180_rad(double input);
	cv::Mat Reverse(cv::Mat nsdt,cv::Mat currentpos);//(double EEx,double EEy,double EEz,cv::Mat nsd,cv::Mat currentpos)
	cv::Mat FindOptimalSol(cv::Mat input,cv::Mat compare);
	void SetPose(double x,double y,double z,double roll,double pitch,double yall,cv::Mat &output);
	cv::Mat Forward(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6);
	cv::Mat Forwardt(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6);
	cv::Mat ForwardJ5(double theta1, double theta2, double theta3, double theta4, double theta5);
	cv::Mat ForwardJ4(double theta1, double theta2, double theta3, double theta4);
	cv::Mat ForwardJ3(double theta1, double theta2, double theta3);
	cv::Mat ForwardJ2(double theta1, double theta2);
	cv::Mat ForwardJ1(double theta1);
	cv::Mat ForwardJ7(double theta1, double theta2);
	cv::Mat ForwardJ8(double theta1, double theta2, double theta3, double theta4);
	cv::Mat ForwardJ9(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6);

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