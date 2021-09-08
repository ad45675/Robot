#include "Tool.h"

// ---------------- SaveData
void SaveData::RecordData(string str, Mat a)
{
	fprintf(SaveDataFile, str.c_str());
	fprintf(SaveDataFile, "\n");

	for (size_t i = 0; i < a.rows; i++)
	{
		for (size_t j = 0; j < a.cols; j++) {
			fprintf(SaveDataFile, "%f\t", a.at<double>(i, j));
		}
		fprintf(SaveDataFile, "\n");
	}


	fprintf(SaveDataFile, "\n");
}

void SaveData::CloseFile()
{

	// --- 關閉txt檔 ---
	{
		/* fclose(
		string filename)		// 檔案路徑及名字*/
	}

	fclose(SaveDataFile);

}

void SaveData::Create(char * path)
{
	// --- 建立txt檔 ---
	{
		/* fopen(
		string filename,		// 檔案路徑及名字
		string mode)			// "w" Open for writing only;*/
	}

	SaveDataFile = fopen(path, "w");
}

// ---------------- ImageTool
vector<Mat> ImageTool::Load(string path, string file, string type, int num, int start)
{
	vector<Mat> Image;
	for (size_t i = start; i < start + num; i++)
	{
		Image.push_back(imread(path + file + to_string(i) + type));
	}

	return Image;
}

vector<Mat> ImageTool::Resize(vector<Mat> Img, double percentage)
{
	vector<Mat> Output;
	for (size_t i = 0; i < Img.size(); i++) {
		Size sz = Img[i].size();
		Size n = Size(sz.width * percentage, sz.height * percentage);
		Mat scale(n, CV_8UC3);
		resize(Img[i], scale, n, (0, 0), (0, 0), INTER_CUBIC);
		Output.push_back(scale);
	}
	return Output;
}

void ImageTool::Show(vector<Mat> Img)
{
	for (size_t i = 0; i < Img.size(); i++) {
		imshow("P" + to_string(i), Img[i]);
	}
}

void ImageTool::Show(Mat Img)
{
		imshow("P" , Img);
}


