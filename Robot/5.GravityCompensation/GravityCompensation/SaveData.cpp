#include "SaveData.h"


//================ 建立實驗資料檔 ================

FILE *SaveDataFile ;

void SaveData_CreateFile( char *path )
{

	SaveDataFile = fopen( path , "w" ) ;

}


//================ 寫入實驗資料檔 ================

void SaveData_Data( double a[] , double b[] , double c[] )
{

	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , a[i] ) ;	
	}
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , b[i] ) ;	
	}
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , c[i] ) ;	
	}
		
	fprintf( SaveDataFile , "\n" ) ;

}


//================ 關閉實驗資料檔 ================

void SaveData_CloseFile()
{

	fclose( SaveDataFile ) ;

}