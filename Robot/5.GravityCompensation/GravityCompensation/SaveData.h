#ifndef SAVEDATA
#define SAVEDATA

#include <stdio.h>

#define AXIS 6

void SaveData_CreateFile( char *path ) ;
void SaveData_Data( double a[] , double b[] , double c[] ) ;
void SaveData_CloseFile() ;

#endif