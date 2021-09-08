#include "Initialization.h"


//============== ����l���שR�O & ��sLSF ==============

void Initialization( double (&PosCmd)[AXIS] )
{

	double Vel[AXIS] = { 0.0 } ;

	for ( int i = 0 ; i < 4 ; i++ ) // LSF��s�ܤֻ�4��
	{

		MotionCard_Encoder( PosCmd ) ; // ���c����(rad)
		Toolbox_LSF( PosCmd , Vel ) ; // ���c�t��(rad/s)

	}

}