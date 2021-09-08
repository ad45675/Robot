#ifndef AXIS
#define AXIS 6
#endif
#ifndef Sol_Num
#define Sol_Num 8
#endif

#ifndef ReadData
#define ReadData

#include <stdio.h>


#include <fstream>
#include <iostream>

using namespace std;


void data(double(&All_Joint)[Sol_Num][AXIS], int(&Object_Num) ,int(&Object_classification)[Sol_Num]) ;


#endif