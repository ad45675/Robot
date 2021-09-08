#ifndef ROBOTMODEL
#define	ROBOTMODEL

#include "Toolbox.h"
#include <math.h>
#include <conio.h>
#include <stdio.h>

#define AXIS 6

void RobotModel_Beta( char *path ) ;
void RobotModel_Feedforward( double Pos[AXIS] , double Vel[AXIS] , double Acc[AXIS] , double (&TorM)[AXIS] , double (&TorN)[AXIS] , double (&TorF)[AXIS] , double (&TorFF)[AXIS] ) ;

#endif