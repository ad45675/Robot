#include "SaveData.h"


//================ �إ߹������� ================

FILE *SaveDataFile ;

void SaveData_CreateFile( char *path )
{

	SaveDataFile = fopen( path , "w" ) ;

}


//================ �g�J�������� ================

void SaveData_Data( double a[] , double b[] , double c[] , double d[] , double e[], int f)
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
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , d[i] ) ;	
	}
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , e[i] ) ;	
	}
	fprintf( SaveDataFile , "%i\t" , f ) ;	
	fprintf( SaveDataFile , "\n" ) ;

}


//================ ������������ ================

void SaveData_CloseFile()
{

	fclose( SaveDataFile ) ;

}