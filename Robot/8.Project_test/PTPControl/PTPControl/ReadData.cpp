#include "ReadData.h"

void data( double (&Target_Joint)[Sol_Num][AXIS], int(&Singularplace)[Sol_Num])
{

		// 讀8組解角度
	fstream Read_Joint("\\\\140.116.215.139/Robot_File/eight_joint.txt",ios::in);
	if (Read_Joint.fail())
	{
		printf("open Read Joint fail");
		system("PAUSE");
	}
	for (int i = 0; i<8; i++)
	{
		for (int j = 0; j<AXIS; j++)
		{
			double temp = 0;
			Read_Joint >> temp;
			Target_Joint[i][j] = temp;
		}
	}
	Read_Joint.close();


	// 讀是否有奇異點
	fstream Read_Singularplace("\\\\140.116.215.139/Robot_File/Singularplace.txt",ios::in);
	if (Read_Joint.fail())
	{
		printf("open Read_Singularplace fail");
		system("PAUSE");
	}
	for (int i = 0; i<AXIS; i++)
	{
		double temp = 0;
		Read_Singularplace >> temp;
		Singularplace[i] = temp;
	}
	Read_Singularplace.close();



}