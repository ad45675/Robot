#ifndef MOTIONCARD
#define MOTIONCARD

#include <IMCDriver.h>
#include <IMCDefine.h>
#include <stdio.h>

#define AXIS 6
#define PI 3.1415926

void MotionCard_OpenCard( int Comport ) ;
void MotionCard_CloseCard() ;
void MotionCard_ServoOn() ;
void MotionCard_ServoOff() ;
void MotionCard_Encoder( double (&RobotPosRad)[AXIS] ) ;
void MotionCard_DAC( double RobotTorCtrl[AXIS] ) ;
void MotionCard_ChangeTimer( TMRISR Timer ) ;

#endif