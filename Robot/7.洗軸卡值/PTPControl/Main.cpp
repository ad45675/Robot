#include "SaveData.h"
#include "MotionCard.h"
#include "Initialization.h"
#include "Toolbox.h"
#include "Protection.h"
#include "Control.h"
#include "PTP.h"
#include "RobotModel.h"
#include <iostream>
#include <conio.h>

#define AXIS 6
#define PI 3.1415926


//======================= Counter & Flag =======================

int Comport = 4 ; // USB comport

int SafetyFlag = 0 ; // �O�@�κX��
int TimerFlag = 0 ; // ���_�����X��
int EndFlag1 = 0 ; // �R�O�����X��1
int EndFlag2 = 0 ; // �R�O�����X��2
int StayCount = 0 ; // �w�ĥέp��


//===============================================================

double InitialPos[AXIS] ; // ��l����
double TargetPos[AXIS] = { 0.0 , 0.7854 , -0.7854 , 0.0 , -0.7854 , 0.0 } ; // �ؼШ���
// Home�I����: { 0.0 , 0.7854 , -0.7854 , 0.0 , -0.7854 , 0.0 } 

double Pos[AXIS] = { 0.0 } ; // �s�X������
double Vel[AXIS] = { 0.0 } ; // LSF�t��
double PosCmd[AXIS] = { 0.0 } ; // ���שR�O
double VelCmd[AXIS] = { 0.0 } ; // �t�שR�O
double AccCmd[AXIS] = { 0.0 } ; // �[�t�שR�O
double TorCtrl[AXIS] = { 0.0 } ; // ��x����T��

double TorM[AXIS] = { 0.0 } ; // �e�X�D�ʤO
double TorN[AXIS] = { 0.0 } ; // �e�X���O&���O
double TorF[AXIS] = { 0.0 } ; // �e�X�����O
double TorF_f[AXIS] = { 0.0 } ; // �o�i��e�X�����O
double TorFF[AXIS] = { 0.0 } ; // �`�e�X��x


//===============================================================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_Stay( TMRINT *pstINTSource ) ;


//===============================================================

int main()
{

	//---------------------- Ū����� ----------------------

	SaveData_CreateFile( "MeasuredData.txt" ) ; // �إ߹�������
	RobotModel_Beta( "Beta.txt" ) ; // ���J�t�ΰѼ�

	//-------------------- �}�ҦU���\�� --------------------

	printf( "Press any key to start !\n" ) ; 
	printf( "Press 'Esc' to end program.\n" ) ;

	while ( _kbhit() == 0 ) ; // �����N��}�l�{��
	
	MotionCard_OpenCard( Comport ) ; // �}�Ҷb�d
	Initialization( PosCmd ) ; // ����l���שR�O & ��sLSF
	MotionCard_ServoOn() ; // Servo On

	
	//-------------------- �����U���\�� --------------------

	MotionCard_ServoOff() ; // Servo Off
	MotionCard_CloseCard() ; // �����b�d
	SaveData_CloseFile() ; // ������������

}


//====================== Step1. PTP��ؼ��I ======================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@


	//------------------------ �I���I���� ------------------------

	PTP_Scurve( InitialPos , TargetPos , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//------------------------- �������_ -------------------------

	if ( EndFlag1 == 1 )
	{

		TimerFlag = 2 ;

	}


	//----------------------- �x�s������ -----------------------

	if (  TimerFlag == 1 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}

}


//======================= Step2. PTP�^���I =======================

void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@


	//------------------------ �I���I���� ------------------------

	PTP_Scurve( TargetPos , InitialPos , PosCmd , VelCmd , AccCmd , EndFlag2 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//----------------------- �x�s������ -----------------------

	if (  TimerFlag == 3 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}


	//------------------------- �������_ -------------------------

	if ( EndFlag2 == 1 )
	{

		TimerFlag = 4 ;

	}

}


//=========================== �w�ĵ{�� ===========================

void _stdcall Timer_Stay( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@


	//--------------------------- ���� ---------------------------

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O


	//------------------------- �������_ -------------------------

	if (  TimerFlag == 0 && StayCount >= 1000 ) // �w��1
	{

		TimerFlag = 1 ;

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // ��l����(Step1.PTP)

		}

	}
	else if (  TimerFlag == 2 && StayCount >= 2000 ) // �w��2
	{

		TimerFlag = 3 ;

	}
	else if (  TimerFlag == 4 && StayCount >= 3000 ) // �w��3
	{

		TimerFlag = 5 ;

	}

	StayCount ++ ;

}
