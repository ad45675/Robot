#include "Setting.h"
#include "Calibrator.h"
#include "Tool.h"

CameraCalibrator Camera;
SaveData Data;
ImageTool I;

int main() {

	// ------------------ Parameter Setting
	//vector<Mat> Image = I.Load("Pic//", "Pic", ".bmp", 12, 1);

	vector<Mat> Image = I.Load("Q2_Image//","Pic",".bmp", 16, 0);
	//I.Show(Image);
	//Camera.setBorderSize(Size(14, 15));
	Camera.setBorderSize(Size(5, 8));
	Data.Create("Data//Camera.txt");

	// ------------------ Image Show
	if (true)
	{
		vector<Mat> Output = I.Resize(Image, 0.2);
		I.Show(Output);
	}

	// ------------------ Add Chess Board Point
	if (false)
	{
		vector<Mat> Chess = Camera.DrawChessboardPoints(Image);
		vector<Mat> Output = I.Resize(Chess, 0.5);
		I.Show(Output);
	}

	// ------------------ Find Intrinsic Matrix
	Camera.Camera_Matrix(Image);
	Camera.Collection();
	Data.RecordData(">>>>> Intrinsic Matrix", Camera.intrinsic);
	Data.RecordData(">>>>> Distortion Coeff", Camera.discoeff);

	// ------------------ Close File
	Data.CloseFile();

	FileStorage camera;

	if (!camera.open("D:/MyRobotGroup/ya0/Kinect/2.HandEyeCalibration/camera.xml", FileStorage::WRITE))
	{
		printf("Open camera fail.\n");
		return false;
	}

	camera << "intrinsic" << Camera.intrinsic << "discoeff" << Camera.discoeff;
	camera.release();



	waitKey(0);
	system("Pause");
	return 0;
}