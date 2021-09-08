#ifndef PTP
#define PTP

#include <stdio.h>
#include <conio.h>
#include <math.h>

#define AXIS 6
#define SamplingTime 0.001

void PTP_Scurve( double InitialPos[AXIS] , double FinalPos[AXIS] , double (&PosCmd)[AXIS] , double (&VelCmd)[AXIS] , double (&AccCmd)[AXIS] , int (&EndFlag) ) ;

#endif