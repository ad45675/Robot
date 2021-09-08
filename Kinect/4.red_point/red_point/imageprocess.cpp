# include"imageprocess.h"
#include "Kinematics.h"

bool imageprocessing::DepthDatafilter(Mat& src_gray16U, int uDepthMax, int uDepthMin) {

	Mat src_gray16U_filt = Mat::zeros(src_gray16U.rows, src_gray16U.cols, CV_16UC1);

	//檢查input type
	CV_Assert(src_gray16U.depth() != sizeof(UINT16*));

	int channels = src_gray16U.channels();
	int nRows = src_gray16U.rows;
	int nCols = src_gray16U.cols * channels;

	/*
	if (src_gray16U.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}
	*/

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

	src_gray16U_filt.copyTo(src_gray16U);  //濾波完的複製過去

	return true;

}

bool imageprocessing::DetectRedObject(Mat& ShowImage, double* RedPointC,Mat intrinsic, Mat discoeff, int redpoint_z)
{

	//resize(ShowImage, ShowImage, Size(1920, 1080));
	int iColorWidth = ShowImage.cols;
	int iColorHeight = ShowImage.rows;
	//int iDepthWidth = ShowImage_gray.cols;
	//int iDepthHeight = ShowImage_gray.rows;

	Mat ROI;
	Mat img_hsv;
	Mat dst;
	Mat r[2];
	Mat mask;
	Mat img_gray;
	Rect rect_External;
	Point2f center;
	float radius;

	Mat erodeStruct = getStructuringElement(MORPH_RECT, Size(5, 5)); //size越大越不破碎	
	Mat erodeStructDepth = getStructuringElement(MORPH_RECT, Size(3, 3)); //size越大越不破碎

	vector< vector< Point> > contours;
	int RoiOffset_X = 0;
	int RoiOffset_Y = 0;
	int depth = 0;
	double X = 0;
	double Y = 0;

	uColorPointNum = iColorWidth * iColorHeight;
	uDepthPointNum = iDepthWidth * iDepthHeight;


	// copy image
	ROI = ShowImage(Rect(RoiOffset_X, RoiOffset_Y, iColorWidth - RoiOffset_X, iColorHeight - RoiOffset_Y)); //切割ROI方塊 (880*1020)

	// find red point
	{
		cvtColor(ROI, img_hsv, COLOR_BGR2HSV);
		dst = Mat::zeros(ROI.rows, ROI.cols, CV_8UC1); //(height,width)
		contours.clear();

		inRange(img_hsv, Scalar(0, 100, 120), Scalar(10, 255, 255), r[0]);	//LowRange
		inRange(img_hsv, Scalar(170, 100, 120), Scalar(180, 255, 255), r[1]);	//HighRange
		mask = r[0] + r[1];

		ROI.copyTo(dst, mask);

		morphologyEx(dst, dst, MORPH_OPEN, erodeStruct); //腐蝕
		cvtColor(dst, img_gray, COLOR_BGR2GRAY);
		findContours(img_gray, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	}

	// morphology depthMap

	{
	//	morphologyEx(ShowImage_gray, ShowImage_gray, MORPH_CLOSE, erodeStructDepth);
	}

	if (contours.size() > 0)
	{
		// draw all red object

		for (int idx = 0; idx < contours.size(); idx++)
		{

			rect_External = boundingRect(contours[idx]); //返回4個值 x，y是矩阵左上点的坐标，w，h是矩阵的宽和高

			if (rect_External.area() > 20)
			{
				minEnclosingCircle(Mat(contours[idx]), center, radius); //尋找包裹輪廓的最小圓
				circle(ShowImage, Point2f(center.x + RoiOffset_X, center.y + RoiOffset_Y), radius, Scalar(0, 0, 255), 3, 8, 0); // 以該物件的最小內圓畫圖
				imshow("P", ShowImage);
				waitKey(0);
			}
		}
	}

	//imshow("ROI", ROI);
	//imshow("color", ShowImage);
	//waitKey(0);
	depth = redpoint_z;
	RedPointC[0] = (center.x + RoiOffset_X - intrinsic.at<double>(0, 2)) * depth / intrinsic.at<double>(0, 0);
	RedPointC[1] = (center.y + RoiOffset_Y - intrinsic.at<double>(1, 2)) * depth / intrinsic.at<double>(1, 1);
	RedPointC[2] = depth ;
	return true;
}

bool imageprocessing::DetectRedObject_depth( Mat& ShowImage_gray)
{

	//resize(ShowImage_gray, ShowImage_gray, Size(1920, 1080));

	int iDepthWidth = ShowImage_gray.cols;
	int iDepthHeight = ShowImage_gray.rows;

	Mat ROI;
	Mat img_hsv;
	Mat dst;
	Mat r[2];
	Mat mask;
	Mat img_gray;
	Rect rect_External;
	Point2f center;
	float radius;

	Mat erodeStruct = getStructuringElement(MORPH_RECT, Size(5, 5)); //size越大越不破碎	
	Mat erodeStructDepth = getStructuringElement(MORPH_RECT, Size(3, 3)); //size越大越不破碎

	vector< vector< Point> > contours;
	int RoiOffset_X = 0;
	int RoiOffset_Y = 0;
	int depth = 0;
	double X = 0;
	double Y = 0;

	uColorPointNum = iColorWidth * iColorHeight;
	uDepthPointNum = iDepthWidth * iDepthHeight;


	// copy image
	ROI = ShowImage_gray(Rect(RoiOffset_X, RoiOffset_Y, iDepthWidth - RoiOffset_X, iDepthHeight - RoiOffset_Y)); //切割ROI方塊 (880*1020)

	// find red point
	{
		cvtColor(ROI, img_hsv, COLOR_BGR2HSV);
		dst = Mat::zeros(ROI.rows, ROI.cols, CV_8UC1); //(height,width)
		contours.clear();

		inRange(img_hsv, Scalar(0, 100, 120), Scalar(10, 255, 255), r[0]);	//LowRange
		inRange(img_hsv, Scalar(170, 100, 120), Scalar(180, 255, 255), r[1]);	//HighRange
		mask = r[0] + r[1];

		ROI.copyTo(dst, mask);

		morphologyEx(dst, dst, MORPH_OPEN, erodeStruct); //腐蝕
		cvtColor(dst, img_gray, COLOR_BGR2GRAY);
		findContours(img_gray, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	}


	if (contours.size() > 0)
	{
		// draw all red object

		for (int idx = 0; idx < contours.size(); idx++)
		{

			rect_External = boundingRect(contours[idx]); //返回4個值 x，y是矩阵左上点的坐标，w，h是矩阵的宽和高

			if (rect_External.area() > 20)
			{
				minEnclosingCircle(Mat(contours[idx]), center, radius); //尋找包裹輪廓的最小圓
				//circle(ShowImage_gray, Point2f(center.x + RoiOffset_X, center.y + RoiOffset_Y), radius, Scalar(0, 0, 255), 3, 8, 0); // 以該物件的最小內圓畫圖
				cout<< center.x <<"  "<<radius<<endl;
			}
		}
	}

	circle(ShowImage_gray, Point2f( 268,127-3.60565), radius, Scalar(0, 0, 255), 3, 8, 0); // 以該物件的最小內圓畫圖

	imshow("ShowImage_gray", ShowImage_gray);
	waitKey(0);

	return true;
}

bool imageprocessing::DepthRaw2DeptMap(Mat src_gray16U, Mat& src_gray, int uDepthMax) {

	// Fix kinect exceed value & color form

	src_gray = Mat::zeros(src_gray16U.rows, src_gray16U.cols, CV_8UC3);

	//檢查input type
	CV_Assert(src_gray16U.depth() != sizeof(UINT16*));

	int channels = src_gray16U.channels(); //1

	int nRows = src_gray16U.rows; //424

	int nCols = src_gray16U.cols * channels; //512

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
				p[3 * j + 1] = 255.0f * (1.0f - ((float)(p_U16[j]) / (float)uDepthMax));
				p[3 * j + 2] = 255.0f * (1.0f - ((float)(p_U16[j]) / (float)uDepthMax));
			}
		}
	}

	return true;

}

vector<Mat> imageprocessing::read_and_show(int start ,int num )
{
	vector<Mat> Image;
	for (size_t i = start; i < start + num; i++)
	{
		Image.push_back(imread("D:\\MyRobotGroup\\ya0\\Kinect\\red_point\\red_point\\Pic\\Color\\Pic" + to_string(i) + ".bmp"));

	}
	/*
	for (size_t i = 0; i < Image.size(); i++) {
		imshow("P" + to_string(i), Image[i]);
		waitKey(0);
	}
    */
	return Image;
}

bool imageprocessing::result_test()
{
	
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

	//----- Hand to Eye
	if (!fs.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\HandToEye.xml", FileStorage::READ))
	{
		printf("Open HandEye File fail...\n");
		system("PAUSE");
	}

	fs["Rotation"] >> R_hand2i;
	fs["Translation"] >> t_hand2i;

	fs.release();
	cout << endl << "HandToEye" << "[Rotation]:\n" << R_hand2i << endl << endl << "[Translation]:\n" << t_hand2i << endl << "------------------------------------" << endl << endl;

	//----- Eye to Hand
	if (!fs.open("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\EyeToHand.xml", FileStorage::READ))
	{
		printf("Open HandEye File fail...\n");
		system("PAUSE");
	}

	fs["Rotation"] >> R_i2hand;
	fs["Translation"] >> t_i2hand;

	fs.release();
	cout << endl << "EyeToHand" << "[Rotation]:\n" << R_i2hand << endl << endl << "[Translation]:\n" << t_i2hand << endl << "------------------------------------" << endl << endl;


	//     相機紅點------------------------------------------------------------------------------
	const int dataNum = 8;
	double RedPoint[dataNum][3] = { 0.0 };

	fstream RedPointFile("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\RedPointC.txt", ios::in);
	if (RedPointFile.fail())
	{
		printf("Open file error...\n");
		system("PAUSE");
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

	//     robot紅點------------------------------------------------------------------------------
	double RobotPos[dataNum][6] = { 0.0 };

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
		}
	}

	RobotPosFile.close();
	FILE* World_coor = fopen("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\World_coor.txt", "w");

	if (World_coor == NULL)
	{
		printf("test_result output file has error...\n");
		system("PAUSE");
	}

	FILE* Camera_coor = fopen("D:\\MyRobotGroup\\ya0\\Kinect\\Data\\Camera_coor.txt", "w");

	if (Camera_coor == NULL)
	{
		printf("test_result output file has error...\n");
		system("PAUSE");
	}


	// 資料提取完畢------------------------------------------------------------------------------

	Kinematics MyRobot;
	vector<Mat> MyRobotPos;
	vector<Mat> MyRobotRedPoint;
	vector<Mat> output;

	//   求 robot 末端點位置
	for (int i = 0; i < dataNum; i++)
	{
		Mat PosMatrix = MyRobot.Forwardt(RobotPos[i][0], RobotPos[i][1] + (PI / 2), RobotPos[i][2], RobotPos[i][3], RobotPos[i][4], RobotPos[i][5]);
		MyRobotPos.push_back(PosMatrix);

		Mat Roboteef_Matrix = Mat::zeros(4, 1, CV_64FC1);
		Roboteef_Matrix.at<double>(0, 0) = PosMatrix.at<double>(0, 0);
		Roboteef_Matrix.at<double>(1, 0) = PosMatrix.at<double>(1, 0);
		Roboteef_Matrix.at<double>(2, 0) = PosMatrix.at<double>(2, 0);
		Roboteef_Matrix.at<double>(3, 0) = 1;

		// 此處在校正的時候盡量讓座標間距小一點
		// 差太多的話會導致校正結果很差
		// ex: Robot -426 mapping到 Camera 430

		Mat RedPointMatrix = Mat::zeros(3, 1, CV_64FC1);
		RedPointMatrix.at<double>(0, 0) = -1 * RedPoint[i][0]; // kinect 攝影機拍攝影像左右顛倒，需 * 負號
		RedPointMatrix.at<double>(1, 0) = RedPoint[i][1];
		RedPointMatrix.at<double>(2, 0) = RedPoint[i][2];

		MyRobotRedPoint.push_back(RedPointMatrix);

		Mat Eye_hand_RotX;
		Eye_hand_RotX = Eye_2_Hand_Matrix(R_i2hand, t_i2hand);

		Mat Hand_eye_RotX;
		Hand_eye_RotX = Hand_2_Eye_Matrix(R_hand2i, t_hand2i);


		Mat cam_matrix = Mat::zeros(4, 1, CV_64FC1);
		cam_matrix.at<double>(0, 0) = RedPointMatrix.at<double>(0, 0);
		cam_matrix.at<double>(1, 0) = RedPointMatrix.at<double>(1, 0);
		cam_matrix.at<double>(2, 0) = RedPointMatrix.at<double>(2, 0);
		cam_matrix.at<double>(3, 0) = 1;
		//cout << "cam_matrix" << ":" << cam_matrix << endl;


		//----------------------------//
		// camera frame to robot frame//
		//----------------------------//

		Mat Eye_hand = Eye_hand_RotX*cam_matrix;

		double x = Eye_hand.at<double>(0, 0);
		double y = Eye_hand.at<double>(1, 0);
		double z = Eye_hand.at<double>(2, 0);
		cout << "----Camera frame to Robot frame (世界座標)----" << endl << endl;
		cout << "Pos " << PosMatrix.at<double>(0, 0) << "  " << PosMatrix.at<double>(1, 0) << "  " << PosMatrix.at<double>(2, 0) << endl;
		cout << "ans " << x << "  " << y << "  " << z << endl;
		cout << "---------------------------------------------------" << endl;

		fprintf(World_coor, "%f\t", x);
		fprintf(World_coor, "%f\t", y);
		fprintf(World_coor, "%f\t", z);
		fprintf(World_coor, "\n");

		//----------------------------//
		// robot frame to camera frame//
		//----------------------------//
	
		Mat Hand_eye = Hand_eye_RotX*Roboteef_Matrix;
		double xc = Hand_eye.at<double>(0, 0);
		double yc = Hand_eye.at<double>(1, 0);
		double zc = Hand_eye.at<double>(2, 0);

		fprintf(Camera_coor, "%f\t", xc);
		fprintf(Camera_coor, "%f\t", yc);
		fprintf(Camera_coor, "%f\t", zc);
		fprintf(Camera_coor, "\n");
		cout << "----Robot frame to Camera frame (相機座標)----" << endl << endl;
		cout << "Pos " << RedPointMatrix.at<double>(0, 0) << "  " << RedPointMatrix.at<double>(1, 0) << "  " << RedPointMatrix.at<double>(2, 0) << endl;
		cout << "ans " << xc << "  " << yc << "  " << zc << endl;
		cout << "---------------------------------------------------" << endl;
		
	}

	return true;
}

Mat imageprocessing::Eye_2_Hand_Matrix(Mat R_i2hand, Mat t_i2hand)
{

	Mat RotX = Mat::zeros(4, 4, CV_64FC1);
	RotX.at<double>(0, 0) = R_i2hand.at<double>(0, 0);
	RotX.at<double>(0, 1) = R_i2hand.at<double>(0, 1);
	RotX.at<double>(0, 2) = R_i2hand.at<double>(0, 2);
	RotX.at<double>(0, 3) = t_i2hand.at<double>(0, 0);

	RotX.at<double>(1, 0) = R_i2hand.at<double>(1, 0);
	RotX.at<double>(1, 1) = R_i2hand.at<double>(1, 1);
	RotX.at<double>(1, 2) = R_i2hand.at<double>(1, 2);
	RotX.at<double>(1, 3) = t_i2hand.at<double>(1, 0);

	RotX.at<double>(2, 0) = R_i2hand.at<double>(2, 0);
	RotX.at<double>(2, 1) = R_i2hand.at<double>(2, 1);
	RotX.at<double>(2, 2) = R_i2hand.at<double>(2, 2);
	RotX.at<double>(2, 3) = t_i2hand.at<double>(2, 0);

	//std::cout <<"=======\n"<< t_i2hand << endl;
	RotX.at<double>(3, 0) = 0;
	RotX.at<double>(3, 1) = 0;
	RotX.at<double>(3, 2) = 0;
	RotX.at<double>(3, 3) = 1;

	return RotX;

}

Mat imageprocessing::Hand_2_Eye_Matrix(Mat R_hand2i, Mat t_hand2i)
{

	Mat RotX = Mat::zeros(4, 4, CV_64FC1);
	RotX.at<double>(0, 0) = R_hand2i.at<double>(0, 0);
	RotX.at<double>(0, 1) = R_hand2i.at<double>(0, 1);
	RotX.at<double>(0, 2) = R_hand2i.at<double>(0, 2);
	RotX.at<double>(0, 3) = t_hand2i.at<double>(0, 0);

	RotX.at<double>(1, 0) = R_hand2i.at<double>(1, 0);
	RotX.at<double>(1, 1) = R_hand2i.at<double>(1, 1);
	RotX.at<double>(1, 2) = R_hand2i.at<double>(1, 2);
	RotX.at<double>(1, 3) = t_hand2i.at<double>(1, 0);

	RotX.at<double>(2, 0) = R_hand2i.at<double>(2, 0);
	RotX.at<double>(2, 1) = R_hand2i.at<double>(2, 1);
	RotX.at<double>(2, 2) = R_hand2i.at<double>(2, 2);
	RotX.at<double>(2, 3) = t_hand2i.at<double>(2, 0);

	//std::cout <<"=======\n"<< t_i2hand << endl;
	RotX.at<double>(3, 0) = 0;
	RotX.at<double>(3, 1) = 0;
	RotX.at<double>(3, 2) = 0;
	RotX.at<double>(3, 3) = 1;

	return RotX;

}
