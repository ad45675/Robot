#include "ReadData.h"

void data( double (&Target_Joint)[AXIS], int(&Singularplace))
{

		// Ū8�ոѨ���
	fstream Read_Joint("\\\\140.116.215.139/Robot_File/JointCmd.txt",ios::in);
	if (Read_Joint.fail())
	{
		printf("open Read Joint fail");
		system("PAUSE");
	}
	
	for (int j = 0; j<AXIS; j++)
	{
		double temp = 0;
		Read_Joint >> temp;
		Target_Joint[j] = temp;
	}
	
	Read_Joint.close();


	// Ū�O�_���_���I
	fstream Read_Singularplace("\\\\140.116.215.139/Robot_File/flag.txt",ios::in);
	if (Read_Joint.fail())
	{
		printf("open Read_Singularplace fail");
		system("PAUSE");
	}
	
	double temp = 0;
	Read_Singularplace >> temp;
	Singularplace = temp;
	
	Read_Singularplace.close();



}