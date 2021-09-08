#ifndef DATAIO// 避免重複引入
#define DATAIO 


#include <iostream>
#include <stdio.h>
#include <fstream>

#ifndef OPENCV_HPP// 避免重複引入
#define OPENCV_HPP 

#include <opencv2/opencv.hpp> 
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#endif

#ifndef NAMESPACE// 避免重複引入
#define NAMESPACE 

using namespace std;
using namespace cv;

#endif

class DataIditor{

public:

	bool ReadData(char *path , double (*a)[] ,int dataNum , int datalength);


};

#endif