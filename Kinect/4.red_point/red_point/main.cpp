// camera_kinect.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <conio.h>

using namespace std;


// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

//user header

#include "imageprocess.h"

using namespace cv;

int KeyBoard_Grab = 0;
bool SavePic = false;
bool detect_red = false;
bool test_result = true;
bool Eye_2_hand = true;


Mat ShowImg;
Mat ShowImg_gray;
int iColorWidth, iColorHeight;
Mat intrinsic;
Mat discoeff;


int main()
{

	imageprocessing imageprocessing;

	FileStorage fs;

	//.......camera parameter
	if (!fs.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\camera.xml", FileStorage::READ))
	{
		printf("open cameracalibration file fail....\n");
		return false;
	}

	fs["intrinsic"] >> intrinsic;
	fs["discoeff"] >> discoeff;
	fs.release();


	cout << "intrinsic" << endl << intrinsic << endl;
	cout << "        " << endl;
	cout << "discoeff" << endl << discoeff << endl;
	cout << "-----------------------------------------------------------------------------" << endl;

	FILE* RedPointCoorC;
	if (detect_red)
	{
		RedPointCoorC = fopen("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\RedPointC_new.txt", "w");

		if (RedPointCoorC == NULL)
		{
			printf("RedPointC output file has error...\n");
			system("PAUSE");
		}
	}


	const int dataNum = 8;
	const int coord = 3;
	double redpoint[dataNum][coord];

	fstream RedPointCoor_Z("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\RedPointC.txt", ios::in);

	if (RedPointCoor_Z.fail())
	{
		printf("Open RedPointCoor_Z file error...\n");
		system("PAUSE");
	}
	for (int i = 0; i < dataNum; i++)
	{
		for (int j = 0; j < coord; j++)
		{
			double temp = 0;
			RedPointCoor_Z >> temp;
			redpoint[i][j] = temp;
		}

	}


	//    讀深度值
	for (int i = 0; i < dataNum; i++)
	{
		cout << "RedPointCoor_Z" << " " << redpoint[i][2] << endl;
	}


	if (detect_red)
	{
		cout << "start detect red point !!!!" << endl;
		Mat ShowImage;
		Mat ShowImage_gray;
		Mat mImg16bit;

		//     讀影像--------------------------------------------------------

		int start = 0;
		int num = 8;
		Mat Image;
		vector<Mat> Image_show;

		//Mat  depth_image = imread("D:\\MyRobotGroup\\ya0\\Kinect\\Pic\\Depth\\R_Pic0.bmp");
		//vector<Mat>  depth_image_show;


		Image_show = imageprocessing.read_and_show(start, num);
		cout << "-----------------------------------------------------------------------------" << endl;
		cout << "影像長寬" << endl << "width" << Image_show[0].cols << "      " << "height" << Image_show[0].rows << endl;
		cout << "      " << endl;

		for (size_t i = 0; i < Image_show.size(); i++)
		{
			Image = Image_show[i];

			// ---------還原原本的size----------//
			//resize(Image, Image, Size(Image.cols * 2, Image.rows * 2));

			int iColorWidth = Image.cols;
			int iColorHeight = Image.rows;

			Image.copyTo(ShowImage);

			double RedPoint[3] = { 0.0 };
			double RedPointDC[3] = { 0.0 };


			imageprocessing.DetectRedObject(ShowImage, RedPoint, intrinsic, discoeff, redpoint[i][2]);
			cout << "Picture" << i << endl;
			printf("RedPoint X : %3.f Y: %3.f Z: %3.f \n", RedPoint[0], RedPoint[1], RedPoint[2]);
			cout << "---------------------------------------------------" << endl;


			//-------------存紅點位置-----------------//
			fprintf(RedPointCoorC, "%f\t", RedPoint[0]);
			fprintf(RedPointCoorC, "%f\t", RedPoint[1]);
			fprintf(RedPointCoorC, "%f\t", RedPoint[2]);
			fprintf(RedPointCoorC, "\n");
			//-------------存紅點位置-----------------//
		}
	}


	if (test_result)
	{
		imageprocessing.result_test();
	}



	system("PAUSE");
	return 0;

}

string currentDateToString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y%m%d_%H_%M_%S", timeinfo);
	string str(buffer);
	return str;
}


