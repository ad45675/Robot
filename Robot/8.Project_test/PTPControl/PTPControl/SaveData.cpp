#include "SaveData.h"


//================ �إ߹������� ================

FILE *SaveDataFile ;

void SaveData_CreateFile( char *path )
{

	SaveDataFile = fopen( path , "w" ) ;

}


//================ �g�J�������� ================

void SaveData_Data( double a[] , double b[] , double c[] , double d[] , double e[] , double f[] , double g[] , double h[] )
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
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , f[i] ) ;	
	}
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , g[i] ) ;	
	}
	for ( int i = 0 ; i < AXIS ; i++ )
	{
		fprintf( SaveDataFile , "%f\t" , h[i] ) ;	
	}
		
	fprintf( SaveDataFile , "\n" ) ;

}


//================ ������������ ================

void SaveData_CloseFile()
{

	fclose( SaveDataFile ) ;

}