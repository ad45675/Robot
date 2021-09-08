#pragma once

#ifndef IMAGE_PROCESS
#define IMAGE_PROCESS


#include <cmath>

#ifndef NAME_SPACE
#define NAME_SPACE

//Define Parameter
#define AXIS			6
using namespace std;

#endif
#ifndef OPEN_CV
#define OPEN_CV

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;


#endif
typedef unsigned short UINT16;
typedef unsigned char UINT;


class imageprocessing {

public:
	bool DepthRaw2DeptMap(Mat, Mat&, int);
	bool DepthDatafilter(Mat&, int, int);
	bool DetectRedObject(Mat&,double*,Mat,Mat,int );
	bool DetectRedObject_depth(Mat&);
	vector<Mat> read_and_show(int , int );
	bool result_test();
	Mat Eye_2_Hand_Matrix(Mat, Mat );
	Mat Hand_2_Eye_Matrix(Mat, Mat);
	//bool DetectRedObject(Mat&, Mat&, Mat, UINT16*, DepthSpacePoint*, double*, double*);


	
public:

	Mat Intrinsic_Matrix;
	Mat distCoeffs;
	Mat  R_hand2i;
	Mat  t_hand2i;
	Mat  R_i2hand;
	Mat  t_i2hand;

	int iColorWidth, iColorHeight;
	UINT uColorPointNum;

	int iDepthWidth, iDepthHeight;
	
	UINT16 uDepthMin, uDepthMax;
	UINT uDepthPointNum;

	Mat Depth_Intrinsic_Matrix;

};








#endif