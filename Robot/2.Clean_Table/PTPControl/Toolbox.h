#ifndef TOOLBOX
#define TOOLBOX

#include <math.h>
#include <conio.h>
#include <stdio.h>

#define AXIS 6
#define SamplingTime 0.001

void Toolbox_LSF( double x[AXIS] , double (&y)[AXIS] ) ;
void Toolbox_Filter( double x[AXIS] , double (&y)[AXIS] ) ;
double Toolbox_sign( double x ) ;

#endif