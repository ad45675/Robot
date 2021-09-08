#include "ImageProcess.h"


bool ImageProcessing::Kinect_ImageInitial(int ColorWidth, int ColorHeight, int DepthWidth, int DepthHeight) {

	// Display parameter

	DisplaySize = Size(300, 300);
	radius = 10;

	FileStorage fs;
	//  讀檔-----------------------------
	if (!fs.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\camera.xml", FileStorage::READ))

	{
		printf("Open CameraCalibration File fail...\n");
		system("PAUSE");
	}

	fs["intrinsic"] >> Intrinsic_Matrix;
	fs["discoeff"] >> distCoeffs;

	fs.release();
	cout << "---------- Parameters ----------\n[Intrinsic Matrix]:\n" << Intrinsic_Matrix << endl;

	/*
	//----- Camera Parameter
	if (!fs.open("D:/MyRobotGroup/Austin/ExperimentResult/CameraCalibration/Camera_parameters-Kinect.xml", FileStorage::READ))
	{
		printf("Open CameraCalibration File fail...\n");
		return false;
	}

	fs["Camera_Matrix"] >> Intrinsic_Matrix;
	fs["Distortion_Coefficients"] >> distCoeffs;

	fs.release();
	*/
	//----- Hand to Eye
	if (!fs.open("D:/MyRobotGroup/ya0/Kinect/Data/HandToEye.xml", FileStorage::READ))
	{
		printf("Open HandEye File fail...\n");
		return false;
	}

	fs["Rotation"] >> R;
	fs["Translation"] >> t;

	fs.release();

	cout << "---------- Parameters ----------\n[Intrinsic Matrix]:\n" << Intrinsic_Matrix << endl << endl << "[Rotation]:\n" << R << endl << endl << "[Translation]:\n" << t << endl << "------------------------------------" << endl << endl;


	//----- Kinect Initial

	iColorWidth = ColorWidth;
	iColorHeight = ColorHeight;
	iDepthWidth = DepthWidth;
	iDepthHeight = DepthHeight;


	Depth_Intrinsic_Matrix = Mat::zeros(3, 3, CV_64FC1);

	return true;
}

bool ImageProcessing::DepthRaw2DepthMap(Mat src_gray16U, Mat& src_gray, int uDepthMax) {

	// Fix kinect exceed value & color form

	src_gray = Mat::zeros(src_gray16U.rows, src_gray16U.cols, CV_8UC3);

	//檢查input type
	CV_Assert(src_gray16U.depth() != sizeof(UINT16*));

	int channels = src_gray16U.channels();
	int nRows = src_gray16U.rows;
	int nCols = src_gray16U.cols* channels;

	if (src_gray16U.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;
	uchar* p;
	UINT16* p_U16;

	for (i = 0; i < nRows; ++i)
	{
		p = src_gray.ptr<uchar>(i);
		p_U16 = src_gray16U.ptr<UINT16>(i);

		for (j = 0; j < nCols; j++)
		{
			if (p_U16[j] == 0 || p_U16[j] > uDepthMax || p_U16[j] < 500)
			{
				continue;
			}
			else
			{
				p[3 * j + 1] = 255.0f *(1.0f - ((float)(p_U16[j]) / (float)uDepthMax));
				p[3 * j + 2] = 255.0f *(1.0f - ((float)(p_U16[j]) / (float)uDepthMax));
			}
		}
	}

	return true;

}

bool ImageProcessing::DepthDatafilter(Mat& src_gray16U, int uDepthMax, int uDepthMin) {

	Mat src_gray16U_filt = Mat::zeros(src_gray16U.rows, src_gray16U.cols, CV_16UC1);

	//檢查input type
	CV_Assert(src_gray16U.depth() != sizeof(UINT16*));

	int channels = src_gray16U.channels();
	int nRows = src_gray16U.rows;
	int nCols = src_gray16U.cols* channels;

	//if (src_gray16U.isContinuous())
	//{
	//	nCols *= nRows;
	//	nRows = 1;
	//}

	int i, j;
	UINT16* p_U16;
	UINT16* p_U16_filt;


	// 内（8个像素）外（16个像素）层阈值
	int innerBandThreshold = 3;
	int outerBandThreshold = 7;
	//

	p_U16 = src_gray16U.ptr<UINT16>(0);
	p_U16_filt = src_gray16U_filt.ptr<UINT16>(0);

	// 第一次濾波

	for (i = 0; i < nRows; ++i)
	{
		for (j = 0; j < nCols; j++)
		{
			int Nowindex = i * iDepthWidth + j;

			if (p_U16[Nowindex] > uDepthMax || p_U16[Nowindex] < uDepthMin)

				p_U16[Nowindex] = 0;
		}

	}


	// 第二次濾波
	for (i = 0; i < nRows; ++i)
	{
		for (j = 0; j < nCols; j++)
		{
			int Nowindex = i * iDepthWidth + j;

			// 我们认为深度值为0的像素即为候选像素
			if (p_U16[Nowindex] == 0)
			{
				// filter collection 用来计算滤波器内每个深度值对应的频度，在后面我们将通过这个数值来确定给候选像素一个什么深度值。
				unsigned long filterCollection[24][2] = { 0 };

				// 内外层框内非零像素数量计数器，在后面用来确定候选像素是否滤波
				int innerBandCount = 0;
				int outerBandCount = 0;

				// 下面的循环将会对以候选像素为中心的5 X 5的像素阵列进行遍历。这里定义了两个边界。如果在
				// 这个阵列内的像素为非零，那么我们将记录这个深度值，并将其所在边界的计数器加一，如果计数器
				// 高过设定的阈值，那么我们将取滤波器内统计的深度值的众数（频度最高的那个深度值）应用于候选
				// 像素上

				for (int yi = -2; yi < 3; yi++)
				{
					for (int xi = -2; xi < 3; xi++)
					{

						// 我们不要xi = 0&&yi = 0的情况，因为此时操作的就是候选像素
						if (xi != 0 || yi != 0)
						{
							// 确定操作像素在深度图中的位置
							int xSearch = j + xi;
							int ySearch = i + yi;

							// 检查操作像素的位置是否超过了图像的边界（候选像素在图像的边缘）
							if (xSearch >= 0 && xSearch < iDepthWidth && ySearch >= 0 && ySearch < iDepthHeight)
							{
								int Searchindex = xSearch + (ySearch * iDepthWidth);
								// 我们只要非零向量
								if (p_U16[Searchindex] != 0)
								{
									// 计算每个深度值的频度
									for (int i = 0; i < 24; i++)
									{
										if (filterCollection[i][0] == p_U16[Searchindex])
										{
											// 如果在 filter collection中已经记录过了这个深度
											// 将这个深度对应的频度加一
											filterCollection[i][1]++;
											break;
										}
										else if (filterCollection[i][0] == 0)
										{
											// 如果filter collection中没有记录这个深度
											// 那么记录
											filterCollection[i][0] = p_U16[Searchindex];
											filterCollection[i][1]++;
											break;
										}
									}

									// 确定是内外哪个边界内的像素不为零，对相应计数器加一
									if (yi != 2 && yi != -2 && xi != 2 && xi != -2)
										innerBandCount++;
									else
										outerBandCount++;
								}
							}
						}
					}
				}

				// 判断计数器是否超过阈值，如果任意层内非零像素的数目超过了阈值，
				// 就要将所有非零像素深度值对应的统计众数
				if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold)
				{
					short frequency = 0;
					short depth = 0;
					// 这个循环将统计所有非零像素深度值对应的众数
					for (int i = 0; i < 24; i++)
					{
						// 当没有记录深度值时（无非零深度值的像素）
						if (filterCollection[i][0] == 0)
						{
							break;

						}
						else if (filterCollection[i][1] > frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}

					p_U16_filt[Nowindex] = depth;
				}
				// 非零向量沒大於閥值
				else
				{
					p_U16_filt[Nowindex] = 0;
				}
			}
			else
			{
				// 如果像素的深度值不为零，保持原深度值
				//continue;
				p_U16_filt[Nowindex] = p_U16[Nowindex];
			}
		}
	}

	src_gray16U_filt.copyTo(src_gray16U);

	return true;

}

bool ImageProcessing::RunTimeTest(void) {

	TickMeter tm;
	tm.start();

	/*

	PASTE YOUR CODE!


	*/

	tm.stop();
	cout << "RunTime(s): " << tm.getTimeSec() << endl;


	return true;


}

bool ImageProcessing::DetectRedObject(Mat& ColorImg, Mat& GrayImage, Mat src_gray16U, UINT16* pDepthPoints, DepthSpacePoint* pPointArray, double* RedPointC, double* RedPointDC)
{
	vector< vector< Point> > contours;
	Mat erodeStruct = getStructuringElement(MORPH_RECT, Size(5, 5)); //size越大越不破碎	
	Mat erodeStructDepth = getStructuringElement(MORPH_RECT, Size(3, 3)); //size越大越不破碎

	Mat img_hsv;
	Mat r[2];
	Mat mask;
	Mat img_gray;
	Mat dst;
	Mat ROI;
	Rect rect_External;
	Point2f center;
	float radius;
	int RoiOffset_X = 500;
	int RoiOffset_Y = 200;
	int RoiOffset_X_ = 300;
	int RoiOffset_Y_ = 0;
	int depth = 0;
	double X = 0;
	double Y = 0;





	// Copy Image		
	ROI = ColorImg(Rect(RoiOffset_X, RoiOffset_Y, iColorWidth - RoiOffset_X- RoiOffset_X_, iColorHeight - RoiOffset_Y - RoiOffset_Y_)); //切割ROI方塊

																										   // find RedPoint
	{
		cvtColor(ROI, img_hsv, COLOR_BGR2HSV);
		dst = Mat::zeros(ROI.rows, ROI.cols, CV_8UC1);
		contours.clear();

		inRange(img_hsv, Scalar(0, 100, 120), Scalar(10, 255, 255), r[0]);	//LowRange
		inRange(img_hsv, Scalar(170, 100, 120), Scalar(180, 255, 255), r[1]);	//HighRange
		mask = r[0] + r[1];

		ROI.copyTo(dst, mask);

		morphologyEx(dst, dst, MORPH_OPEN, erodeStruct);
		cvtColor(dst, img_gray, COLOR_BGR2GRAY);
		findContours(img_gray, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	}


	// morphology depthMap
	{
		morphologyEx(GrayImage, GrayImage, MORPH_CLOSE, erodeStructDepth);
	}

	// If Find Any Red Object
	if (contours.size() > 0)	// 有找到任何的紅色物件
	{
		// Draw All Red Object
		for (int idx = 0; idx < contours.size(); idx++)
		{
			rect_External = boundingRect(contours[idx]);

			if (rect_External.area() > 20)
			{
				minEnclosingCircle(Mat(contours[idx]), center, radius);
				circle(ColorImg, Point2f(center.x + RoiOffset_X, center.y + RoiOffset_Y), radius, Scalar(0, 0, 255), 3, 8, 0); // 以該物件的最小內圓畫圖
			}
		}


		// Map color image to gray image & retrieve red point depth

		size_t idx = floor(center.y + RoiOffset_Y) * iColorWidth + floor(center.x + RoiOffset_X); // 最後一個紅色object座標
		const DepthSpacePoint& rPoint = pPointArray[idx];

		if (rPoint.X >= 0 && rPoint.X < iDepthWidth && rPoint.Y >= 0 && rPoint.Y < iDepthHeight)
		{
			Point2f Graycenter(rPoint.X, rPoint.Y);
			size_t gray_idx = floor(rPoint.Y) * iDepthWidth + floor(rPoint.X);

			circle(GrayImage, Graycenter, radius, Scalar(0, 0, 255), 3, 8, 0);
			depth = pDepthPoints[gray_idx];

			RedPointC[0] = (center.x + RoiOffset_X - Intrinsic_Matrix.at<double>(0, 2)) * depth / Intrinsic_Matrix.at<double>(0, 0);
			RedPointC[1] = (center.y + RoiOffset_Y - Intrinsic_Matrix.at<double>(1, 2)) * depth / Intrinsic_Matrix.at<double>(1, 1);
			RedPointC[2] = depth;

			RedPointDC[0] = (Graycenter.x - Depth_Intrinsic_Matrix.at<double>(0, 2)) * depth / Depth_Intrinsic_Matrix.at<double>(0, 0);
			RedPointDC[1] = (Graycenter.y - Depth_Intrinsic_Matrix.at<double>(1, 2)) * depth / Depth_Intrinsic_Matrix.at<double>(1, 1);
			RedPointDC[2] = depth;

		}
	}

	resize(ROI, ROI, Size(0.5 * ROI.cols, 0.5 * ROI.rows));
	imshow("ROI", ROI);

	return true;
}