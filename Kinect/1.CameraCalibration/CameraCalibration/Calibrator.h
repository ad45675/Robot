#ifndef CALIBRATOR
#define CALIBRATOR

#include "Setting.h"


class CameraCalibrator {
private:
	Size Boardsize;

public:
	bool exist = false;
	vector<Mat> extrinsic;
	Mat intrinsic, discoeff, dst;

public:
	void clear();
	void Camera_Matrix(vector<Mat> &A);
	void setBorderSize(const Size &borderSize);
	void extrinsicvec2mat(vector<Mat> rvecs, vector<Mat> tvecs);
	void Collection();
	Mat drawChessboardPoints(Mat A);
	vector<Mat> DrawChessboardPoints(vector<Mat> Img);
};
#endif // !
