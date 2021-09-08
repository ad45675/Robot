#include "Initialization.h"


//============== 給初始角度命令 & 更新LSF ==============

void Initialization( double (&PosCmd)[AXIS] )
{

	double Vel[AXIS] = { 0.0 } ;

	for ( int i = 0 ; i < 4 ; i++ ) // LSF更新至少需4次
	{

		MotionCard_Encoder( PosCmd ) ; // 機構角度(rad)
		Toolbox_LSF( PosCmd , Vel ) ; // 機構速度(rad/s)

	}

}