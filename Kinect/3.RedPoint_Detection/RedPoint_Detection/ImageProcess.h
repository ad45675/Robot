#pragma once
#ifndef IMAGEPROCESS
#define IMAGEPROCESS

#include <cmath>
#include "Setting.h"


class ImageProcessing {


public:
	bool Kinect_ImageInitial(int, int, int, int);
	bool RunTimeTest(void);
	bool DepthRaw2DepthMap(Mat, Mat&, int);
	bool DepthDatafilter(Mat&, int, int);
	bool DetectRedObject(Mat&, Mat&, Mat, UINT16*, DepthSpacePoint*, double*, double*);

public:
	Size DisplaySize;
	Mat Intrinsic_Matrix;
	Mat Depth_Intrinsic_Matrix;
	Mat distCoeffs;
	Mat R, t;
	double radius;

	int iColorWidth, iColorHeight;
	int iDepthWidth, iDepthHeight;


	int controlPointNum;


};



#endif