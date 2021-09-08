//=======================================================================================
// Modified by ya0 @ 2021/03/05
// This program use kinect to caputre red point attached to the end-effector
// Use least square method to compute transformation between robot and camera coordinate
//=======================================================================================


/** Please remember to hit "EXC" before close the program, or the file may not been record. **/

// Standard Library
#include <iostream>
#include <stdio.h>
#include <windows.h>
#include <conio.h>
#include <array>
#include <fstream>
#include "Setting.h"
#include "Tool.h"

#include "DataIO.h"
#include "Kinematics.h"

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

//Define Parameter
#define AXIS			6

using namespace std;
using namespace cv;

bool SavePic = false;
int KeyBoard_Grab = 0;
int NumOfSavePic = 0;
double Pos[AXIS] = { 0.0 };
ImageTool I;
bool show = false;

string int2str(int &i) {
	string s;
	stringstream ss(s);
	ss << i;
	return ss.str();
}

int main(int argc, char** argv)
{
	//-------------------- Start Calibration --------------------

	//---------- ReadFile
	Mat intrinsic;
	Mat distortion;
	Mat Depth_Intrinsic_Matrix;
	Mat Depth_Distortion;

	{
		FileStorage fs;

		if (!fs.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\camera.xml", FileStorage::READ))
		{
			printf("Open CameraCalibration File fail.\n");
			return false;
		}
		
		fs["intrinsic"] >> intrinsic;
		fs["discoeff"] >> distortion;

		fs.release();

		/*if (!fs.open("D:/MyRobotGroup/Austin/ExperimentResult/CameraCalibration/DepthCamera_parameters-Kinect.xml", FileStorage::READ))
		{
			printf("Open DepthCameraCalibration File fail.\n");
			return false;
		}

		fs["Camera_Matrix"] >> Depth_Intrinsic_Matrix;
		fs["Distortion_Coefficients"] >> Depth_Distortion;

		fs.release();
		

		cout << "----------Kinect-Parameter----------\n[Intrinsic Matrix]: " <<endl<< intrinsic << endl << endl << "[Distortion]: " <<endl<< distortion << endl;
	  */
	}
	
	
	const int dataNum =6;

	double RobotPos[dataNum][6] = { 0.0 };
	// --------------------------robot joint------------------------------------//
	fstream RobotPosFile("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\robot_pos.txt", ios::in);


	if (RobotPosFile.fail())
	{
		printf("Open file error...\n");
		return -1;
	}
	
	for (int i = 0; i < dataNum; i++)
	{
		for (int j = 0; j < AXIS; j++)
		{
			double temp = 0;
			RobotPosFile >> temp;
			RobotPos[i][j] = temp;
			//cout << RobotPos[i][j] << endl;
		}
	}

	RobotPosFile.close();

	// --------------------------red point------------------------------------//
	double RedPoint[dataNum][3] = { 0.0 };

	fstream RedPointFile("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\RedPointC.txt", ios::in);
	if (RedPointFile.fail())
	{
		printf("Open file error...\n");
		return -1;
	}

	for (int i = 0; i < dataNum; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double temp = 0;
			RedPointFile >> temp;
			RedPoint[i][j] = temp;
		}
	}
	RedPointFile.close();
// --------------------------robot 末端點位置--------------------------------//

	FILE* robot_EEF = fopen("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\robot_EEF.txt", "w");

	if (robot_EEF == NULL)
	{
		printf("robot_EEF output file has error...\n");
		system("PAUSE");
	}


	cout << "----------Record-Point----------" << endl;

	Kinematics MyRobot;
	vector<Mat> MyRobotPos;
	vector<Mat> MyRobotRedPoint;

	for (int i = 0; i < dataNum; i++)
	{	
	
		Mat PosMatrix = MyRobot.Forwardt(RobotPos[i][0], RobotPos[i][1] + (PI/2), RobotPos[i][2] , RobotPos[i][3], RobotPos[i][4], RobotPos[i][5]);
		MyRobotPos.push_back(PosMatrix);

		
		fprintf(robot_EEF, "%f\t", PosMatrix.at<double>(0,0));
		fprintf(robot_EEF, "%f\t", PosMatrix.at<double>(1,0));
		fprintf(robot_EEF, "%f\t", PosMatrix.at<double>(2,0));
		fprintf(robot_EEF, "\n");
		

		// 此處在校正的時候盡量讓座標間距小一點
		// 差太多的話會導致校正結果很差
		// ex: Robot -426 mapping到 Camera 430

		Mat RedPointMatrix = Mat::zeros(3, 1, CV_64FC1);
		RedPointMatrix.at<double>(0,0) = -1 * RedPoint[i][0] ; // kinect 攝影機拍攝影像左右顛倒，需 * 負號
		RedPointMatrix.at<double>(1,0) = RedPoint[i][1] ;
		RedPointMatrix.at<double>(2,0) = RedPoint[i][2] ;
		
		MyRobotRedPoint.push_back(RedPointMatrix);


		cout << "Camera X: " << RedPointMatrix.at<double>(0, 0) << " Camera y: " << RedPointMatrix.at<double>(1, 0) << " Camera z: " << RedPointMatrix.at<double>(2, 0) << " MyRobot X: " << PosMatrix.at<double>(0.0) << " MyRobot Y: " << PosMatrix.at<double>(1.0) << " MyRobot Z: " << PosMatrix.at<double>(2.0) << endl;
	}
	

	// USING LEAST SQUARE METHOD

	Mat CA = Mat::zeros(3, 1, CV_64FC1);
	Mat CB = Mat::zeros(3, 1, CV_64FC1);


	for (int i = 0; i < dataNum; i++)
	{
		CB += MyRobotPos[i];
		CA += MyRobotRedPoint[i];
	}

	CA /= dataNum;
	CB /= dataNum;

	Mat M = Mat::zeros(3, 3, CV_64FC1);
	Mat N = Mat::zeros(3, 3, CV_64FC1);

	for (int i = 0; i < dataNum; i++)
	{
		Mat CovarianceA = MyRobotRedPoint[i] - CA;
		Mat CovarianceB = MyRobotPos[i] - CB;

		M += CovarianceB*(CovarianceA.t());
		N += CovarianceA*(CovarianceB.t());
	}

	vector<Mat> R, t;

	for (int i = 0; i < 2; i++)
	{
		Mat S, U, V;

		if(i==0)
			SVD::compute(M, S, U, V, 0);
		else
			SVD::compute(N, S, U, V, 0);

		Mat D;
		D = Mat::zeros(3, 3, CV_64FC1);
		D.at<double>(0, 0) = 1.0;
		D.at<double>(1, 1) = 1.0;
		D.at<double>(2, 2) = determinant(U*V);

		if (i == 0)	// Eye to Hnad
		{
			R.push_back(U*D*V);
			t.push_back(CB - R[i]*CA);
		}
		
		else // Hand to Eye
		{
			R.push_back(U*D*V);
			t.push_back(CA - R[i] * CB);
		}


		cout << R[i] << endl << t[i] << endl;
	}
	


	FileStorage HandEyeTransform;

	if (!HandEyeTransform.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\EyeToHand.xml", FileStorage::WRITE))
	{
		printf("Open HandEyeTransform fail.\n");
		return false;
	}

	HandEyeTransform << "Rotation" << R[0] << "Translation" << t[0];
	HandEyeTransform.release();


	if (!HandEyeTransform.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\HandToEye.xml", FileStorage::WRITE))
	{
		printf("Open HandEyeTransform fail.\n");
		return false;
	}

	HandEyeTransform << "Rotation" << R[1] << "Translation" << t[1];
	HandEyeTransform.release();



	system("PAUSE");
	return 0;
}