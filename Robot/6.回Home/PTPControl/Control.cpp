#include "Control.h"


//================ Feedback Control ================

void Control_Feedback( double Pos[AXIS] , double Vel[AXIS] , double PosCmd[AXIS] , double (&TorCtrl)[AXIS] )
{

	double InertiaMoment[AXIS] = { 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 } ; // ���c�D�q�`��

	double Kp[AXIS] = { 250.0 , 250.0 , 200.0 , 250.0 , 150.0 , 350.0 } ;
	double Kv[AXIS] = { 100.0 , 100.0 , 100.0 , 40.0 , 30.0 , 5.0 } ; 
	
	//double w[AXIS] = { 230.0 , 180.0 , 180.0 , 40.0 , 25.0 , 7.0 } ; // �۵M�W�v { 250 , 200 , 200 , 45 , 30 , 8 }
	//double z[AXIS] = { 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 } ; // �����Y��

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		TorCtrl[i] = InertiaMoment[i] * ( Kv[i] * ( Kp[i] * ( PosCmd[i] - Pos[i] ) - Vel[i] ) ) ; // ��x����R�O(Nm)

		//double Kp = w[i] / ( 2 * z[i] ) ;
		//double Kv = 2 * z[i] * w[i] ;
		//AccCtrl[i] = Kv * ( Kp * ( PosCmd[i] - Pos[i] ) - Vel[i] ) ;

	}

}