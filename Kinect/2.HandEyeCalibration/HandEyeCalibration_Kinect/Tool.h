#ifndef TOOL
#define TOOL

#include <iostream>
#include "Setting.h"

using namespace std;

class SaveData {
private:
	FILE *SaveDataFile;

public:
	void Create(char *path);
	void RecordData(string str, Mat a);
	void CloseFile();
};

class ImageTool {
public:
	vector<Mat> Load(string path, string file, string type, int num, int start);
	vector<Mat> Resize(vector<Mat> Img, double percentage);
	void Show(vector<Mat> Img);
	void Show(Mat Img);


};

#endif // !TOOL
