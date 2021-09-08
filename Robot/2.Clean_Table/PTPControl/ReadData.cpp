#include "ReadData.h"

void data( double (&All_Joint)[Sol_Num][AXIS], int &Object_Num, int (&Object_classification)[Sol_Num])
{
	
	// 讀有幾組解
	fstream Read_Object_Num("\\\\140.116.215.139/Robot_File/Object_Num.txt",ios::in);
	if (Read_Object_Num.fail())
	{
		printf("open Read_Object_Num fail");
		system("PAUSE");
	}

	double temp = 0;
	Read_Object_Num >> temp;
	Object_Num = temp;
	Read_Object_Num.close();
	

	// 讀為哪一類
	fstream Read_Object_classification("\\\\140.116.215.139/Robot_File/Object_classification.txt",ios::in);
	if (Read_Object_classification.fail())
	{
		printf("open Read_Object_classification fail");
		system("PAUSE");
	}

	
	
	for (int i = 0; i<Object_Num; i++)
	{
		double temp = 0.0;
		//int temp = 0;
		Read_Object_classification >> temp;
		Object_classification[i] = (int)temp;
		cout<<"Object_classification" << Object_classification[i] << endl;
	}
	
	Read_Object_classification.close();
	
	

	// 讀所有角度位置
	fstream Read_Joint("\\\\140.116.215.139/Robot_File/JointCmd_Clean.txt",ios::in);
	if (Read_Joint.fail())
	{
		printf("open Read Joint fail");
		system("PAUSE");
	}

	for (int i = 0; i < Object_Num; i++)
	{
		for (int j = 0; j<AXIS; j++)
		{
			double temp = 0;
			Read_Joint >> temp;
			All_Joint[i][j] = temp;
		}
	}

	Read_Joint.close();
	




}