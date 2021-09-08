// USER Header
#include "Kinect_operation.h"
#include "ImageProcess.h"
#include "Setting.h"
#include <direct.h>


// Parameter
bool SavePic = false;
int KeyBoard_Grab = 0;
int NumOfSavePic = 0;
Kinect kinect;
ImageProcessing Imageiditor;
string c, d;

double RedPointC[3] = { 0.0 };
double RedPointDC[3] = { 0.0 };

bool Color = true;
bool Depth = true;
bool Red = true;

// -------------------------- Functions
string currentDateToString();

int main(int argc, char** argv)
{
	// --------------------- OpenKinect
	if (!kinect.OpenKinect())
		return -1;

	// --------------------- Image Initial
	cout << "kinect:" << kinect.iColorHeight << endl;
	if (!Imageiditor.Kinect_ImageInitial(kinect.iColorWidth, kinect.iColorHeight, kinect.iDepthWidth, kinect.iDepthHeight))
		return -1;

	//  -------------------- Kinect OpenCV Initialization -------------------
	Mat mColorImg = Mat::zeros(kinect.iColorHeight, kinect.iColorWidth, CV_8UC4);
	Mat mImg16bit(kinect.iDepthHeight, kinect.iDepthWidth, CV_16UC1);
	Mat mDepthImg(kinect.iDepthHeight, kinect.iDepthWidth, CV_8UC3);

	//  ------------------ Make folder ------------------
	string str = "Pic//";
	str += currentDateToString();
	
	_mkdir(str.c_str());
	
	if (Color)
	{
		c = str + "//Color";   
		_mkdir(c.c_str());
	}
	
	if (Depth)
	{
		d = str + "//Depth";	
		_mkdir(d.c_str());
	}

	// -------------- Get Depth Intrinsic ---------------

	while (true)
	{
		kinect.pCoordinateMapper->GetDepthCameraIntrinsics(kinect.m_pCameraIntrinsics);

		if (kinect.m_pCameraIntrinsics->FocalLengthX)
		{
			cout << "FocalLengthX : " << kinect.m_pCameraIntrinsics->FocalLengthX << endl; //摄像机的X焦距，以像素为单位
			cout << "FocalLengthY : " << kinect.m_pCameraIntrinsics->FocalLengthY << endl; //相机的Y焦距，以像素为单位
			cout << "PrincipalPointX : " << kinect.m_pCameraIntrinsics->PrincipalPointX << endl;//相机在X维中的主要点，以像素为单位。
			cout << "PrincipalPointY : " << kinect.m_pCameraIntrinsics->PrincipalPointY << endl; //相机在Y轴上的主要点，以像素为单位。
			cout << "RadialDistortionFourthOrder : " << kinect.m_pCameraIntrinsics->RadialDistortionFourthOrder << endl;//相机的四阶径向畸变参数。
			cout << "RadialDistortionSecondOrder : " << kinect.m_pCameraIntrinsics->RadialDistortionSecondOrder << endl;//相机的二阶径向畸变参数。
			cout << "RadialDistortionSixthOrder : " << kinect.m_pCameraIntrinsics->RadialDistortionSixthOrder << endl;//摄像机的六阶径向畸变参数。

			Imageiditor.Depth_Intrinsic_Matrix.at<double>(0, 0) = kinect.m_pCameraIntrinsics->FocalLengthX;
			Imageiditor.Depth_Intrinsic_Matrix.at<double>(1, 1) = kinect.m_pCameraIntrinsics->FocalLengthY;
			Imageiditor.Depth_Intrinsic_Matrix.at<double>(0, 2) = kinect.m_pCameraIntrinsics->PrincipalPointX;
			Imageiditor.Depth_Intrinsic_Matrix.at<double>(1, 2) = kinect.m_pCameraIntrinsics->PrincipalPointY;
			Imageiditor.Depth_Intrinsic_Matrix.at<double>(2, 2) = 1;

			cout << "---------- Parameters ----------\n[Depth Intrinsic Matrix]:\n" << Imageiditor.Depth_Intrinsic_Matrix << endl;

		}
		else
		{
			continue;
		}

		break;
	}

	// ------------------- Data IO ---------------------
	string filename = str + "//RedPointC.txt";
	string filename_D = str + "//RedPointDC.txt";
	FILE* RedPointCoorC = fopen(filename.c_str(), "w");
	FILE* RedPointCoorDC = fopen(filename_D.c_str(), "w");

	if (RedPointCoorC == NULL)
	{
		printf("RedPointC output file has error...\n");
		return -1;
	}

	// -------------- Start Record Point ---------------
	cout << "Start Recording Point\n";
	while (true)
	{
		if (_kbhit())
		{
			KeyBoard_Grab = _getch();

			if (KeyBoard_Grab == 0x1b)
				break;

			if (KeyBoard_Grab == 0x20)
				SavePic = true;

			KeyBoard_Grab = 0;

		}

		if (kinect.Capture_Color(mColorImg) && kinect.Capture_Depth(mImg16bit))
		{
			Mat ShowImg; Mat ShowImg_gray; Mat ShowImg_R; Mat ShowImg_gray_R;

			// -------------------- 深度資料處理 --------------------

			{
				Imageiditor.DepthDatafilter(mImg16bit, kinect.uDepthMax, kinect.uDepthMin);
			}

			// ------------------- 彩色&深度影像處理 -------------------

			{
				Imageiditor.DepthRaw2DepthMap(mImg16bit, mDepthImg, kinect.uDepthMax);


				mColorImg.copyTo(ShowImg);
				mDepthImg.copyTo(ShowImg_gray);

			}

			// ------- 找紅點 -------

			{
				mColorImg.copyTo(ShowImg_R);
				mDepthImg.copyTo(ShowImg_gray_R);
				if (kinect.pCoordinateMapper->MapColorFrameToDepthSpace(kinect.uDepthPointNum, kinect.pDepthPoints, kinect.uColorPointNum, kinect.pPointArray) == S_OK)
				{
					
					Imageiditor.DetectRedObject(ShowImg_R, ShowImg_gray_R, mImg16bit, kinect.pDepthPoints, kinect.pPointArray, RedPointC, RedPointDC);

				}

				printf("RedPoint X : %3.f Y: %3.f Z: %3.f DX: %3.f DY: %3.f DZ: %3.f\r", RedPointC[0], RedPointC[1], RedPointC[2], RedPointDC[0], RedPointDC[1], RedPointDC[2]);
			}


			// -------------------- Show Result --------------------

			{
				//resize(ShowImg, ShowImg, Size(0.5 * kinect.iColorWidth, 0.5 * kinect.iColorHeight));
				//resize(ShowImg_gray, ShowImg_gray, Size(0.5 * kinect.iDepthWidth, 0.5 * kinect.iDepthHeight));
				resize(ShowImg_R, ShowImg_R, Size(0.5 * kinect.iColorWidth, 0.5 * kinect.iColorHeight));
				resize(ShowImg_gray_R, ShowImg_gray_R, Size(0.5 * kinect.iDepthWidth, 0.5 * kinect.iDepthHeight));

				imshow("Color Map", ShowImg_R);
				imshow("Depth Map", ShowImg_gray_R);

				waitKey(10);

			}

			// -------------------- Save Result --------------------

			{
				if (SavePic)
				{
					cout << "Capture Image....\n";
					if (Color)
					{
						imwrite(c + "\\Pic" + to_string(NumOfSavePic) + ".bmp", ShowImg);
						if (Red) imwrite(c + "\\R_Pic" + to_string(NumOfSavePic) + ".bmp", ShowImg_R);
					}
					if (Depth)
					{
						imwrite(d + "\\Pic" + to_string(NumOfSavePic) + ".bmp", ShowImg_gray);
						if (Red) imwrite(d + "\\R_Pic" + to_string(NumOfSavePic) + ".bmp", ShowImg_gray_R);

					}
					
					// Record RedPointC
					fprintf(RedPointCoorC, "%f\t%f\t%f\t", RedPointC[0], RedPointC[1], RedPointC[2]);
					fprintf(RedPointCoorDC, "%f\t%f\t%f\t", RedPointDC[0], RedPointDC[1], RedPointDC[2]);
					fprintf(RedPointCoorC, "\n");
					fprintf(RedPointCoorDC, "\n");

					NumOfSavePic++;
					SavePic = false;
				}

			}
		}
	}

	//Close Kinect
	if (!kinect.CloseKinect())
		return -1;

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