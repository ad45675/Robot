#ifndef SAVEDATA
#define SAVEDATA

#include <stdio.h>

#define AXIS 6

void SaveData_CreateFile( char *path ) ;
void SaveData_Data( double a[] , double b[] , double c[] , double d[] , double e[] , double f[] , double g[] , double h[] ) ;
void SaveData_CloseFile() ;

#endif