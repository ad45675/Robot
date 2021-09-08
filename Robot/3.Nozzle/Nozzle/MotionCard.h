#ifndef IMC_INCLUDE // Á×§K­«½Æ¤Þ¤J
#define IMC_INCLUDE


#endif
#include <stdio.h>
#include <conio.h>
#include "IMCDefine.h"
#include "IMCDriver.h"
#include "Setting.h"


void MotionCard_OpenCard() ;
void MotionCard_CloseCard( ) ;
void MotionCard_ChangeTimer( TMRISR Timer ) ;
void MotionCard_ADC(float(&ADCVoltage)[8]);