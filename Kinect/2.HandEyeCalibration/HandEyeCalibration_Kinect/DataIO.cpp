#include "DataIO.h"



//===================== Åª¨ú¸ê®Æ =====================

bool DataIditor::ReadData(char *path , double(*a)[],int dataNum , int datalength)
{

	fstream fileProcess(path, ios::in);

	if (fileProcess.fail())
	{
		printf("Open file error...\n");
		return false;
	}

	while (true)
	{
		
		for (int i = 0; i < dataNum; i++)
		{
			vector<double> DataArray ;

			for (int j = 0; j < datalength; j++)
			{
				double data = 0;

				fileProcess >> data;

				DataArray.push_back(data) ;


			}

		}
	}

	fileProcess.close();

}


