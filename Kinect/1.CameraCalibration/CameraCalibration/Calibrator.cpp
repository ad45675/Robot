#include "Calibrator.h"

Mat CameraCalibrator::drawChessboardPoints(Mat A)
{
	Mat Img;
	A.copyTo(Img);
	vector<Point2f> corners;
	Mat targetGray;
	cvtColor(Img, targetGray, COLOR_BGR2GRAY);
	bool ret = findChessboardCorners(targetGray, Boardsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
	if (ret) {
		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(targetGray, corners, Size(5, 5), Size(-1, -1), param);
		drawChessboardCorners(Img, Boardsize, corners, ret);
		cout << "Successfully Find ChessBoard" << endl;
	}
	else
		cout << "UnSuccessfully Find ChessBoard" << endl;

	return Img;
}

vector<Mat> CameraCalibrator::DrawChessboardPoints(vector<Mat> Img)
{
	vector<Mat> Result;
	for (int i = 0; i < Img.size(); i++)
	{
		Result.push_back(drawChessboardPoints(Img[i]));
	}
	return Result;
}

void CameraCalibrator::clear()
{
	extrinsic.clear();
}

void CameraCalibrator::Camera_Matrix(vector<Mat> &A) {
	exist = true;
	vector<vector<Point2f>>m_srcPoints;
	vector<vector<Point3f>>m_dstPoints;

	vector<Point2f> srcCandidateCorners;
	vector<Point3f> dstCandidateCorners;
	Size imageSize = A[0].size();
	Mat map1, map2;
	Mat targetGray;
	for (int i = 0; i < Boardsize.height; i++) {
		for (int j = 0; j < Boardsize.width; j++) {
			dstCandidateCorners.push_back(Point3f(i, j, 0.0f));
		}
	}

	for (size_t i = 0; i < A.size(); i++) {
		bool ret = findChessboardCorners(A[i], Boardsize, srcCandidateCorners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (ret) {
			TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
			cvtColor(A[i], targetGray, COLOR_BGR2GRAY);
			cornerSubPix(targetGray, srcCandidateCorners, Size(5, 5), Size(-1, -1), param);

			if (srcCandidateCorners.size() == Boardsize.area()) {
				m_srcPoints.push_back(srcCandidateCorners);
				m_dstPoints.push_back(dstCandidateCorners);
			}
			cout << ">> Pic " << i << "\tSuccessfully Find ChessBoard" << endl;
		}
		else {
			cout << ">> Pic " << i << "\tUnSuccessfully Find ChessBoard" << endl;
		}

	}

	vector<Mat> rvecs, tvecs;
	calibrateCamera(m_dstPoints, m_srcPoints, imageSize, intrinsic, discoeff, rvecs, tvecs);
	extrinsicvec2mat(rvecs, tvecs);
	initUndistortRectifyMap(intrinsic, discoeff, Mat(), Mat(), imageSize, CV_32F, map1, map2);
}

void CameraCalibrator::extrinsicvec2mat(vector<Mat> rvecs, vector<Mat> tvecs)
{
	for (size_t i = 0; i < rvecs.size(); i++) {
		Mat rvecsT, RT;
		Rodrigues(rvecs[i], rvecsT);
		hconcat(rvecsT, tvecs[i], RT);
		extrinsic.push_back(RT);
	}
}

void CameraCalibrator::setBorderSize(const Size &borderSize) {
	Boardsize = borderSize;
}


void CameraCalibrator::Collection()
{
	cout << "------------------ Intrinsic Matrix -------------------\n";
	cout << intrinsic << endl;

	cout << "------------------ Distortion Coeff -------------------\n";
	cout << discoeff << endl;
}
