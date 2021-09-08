#include "Nozzle.h"
#include "MotionCard.h"

int suck  = 7;
int release = 6;

void Nozzle_Suck()
{

	IMC_DAC_SetOutputVoltage(7, 6, 0);

}

void Nozzle_Release()
{

	IMC_DAC_SetOutputVoltage(release, 6, 0);

}