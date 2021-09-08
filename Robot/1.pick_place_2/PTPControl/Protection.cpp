#include "Protection.h"


//================== �t�׫O�@�{�� ==================

const double GearRatio[AXIS] = { 120.0 , 120.0 , 120.0 , 102.0 , 80.0 , 51.0 } ; // ��t��

void Protection( double Vel[AXIS] , int (&SafetyFlag) )
{

	for(int i = 0 ; i < AXIS ; i++ )
	{

		double VelBound = ( ( 4000 / 9.55 ) / GearRatio[i] ) ; // �t�׭���(rad/s)

		if ( abs( Vel[i] ) > VelBound )
		{
			SafetyFlag = 1 ;
		}
		
	}

}