#include "SaveData.h"
#include "MotionCard.h"
#include "Initialization.h"
#include "Toolbox.h"
#include "Protection.h"
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
double cnt2 = 0 ; // �s��ƭp�ƾ�


//===============================================================

double Pos[AXIS] = { 0.0 } ; // �s�X������
double Vel[AXIS] = { 0.0 } ; // LSF�t��
double PosCmd[AXIS] = { 0.0 } ; // ���שR�O
double VelCmd[AXIS] = { 0.0 } ; // �t�שR�O
double AccCmd[AXIS] = { 0.0 } ; // �[�t�שR�O
double TorCtrl[AXIS] = { 0.0 } ; // ��x����T��

double TorM[AXIS] = { 0.0 } ; // �e�X�D�ʤO
double TorN[AXIS] = { 0.0 } ; // �e�X���O&���O
double TorF[AXIS] = { 0.0 } ; // �e�X�����O
double TorFF[AXIS] = { 0.0 } ; // �`�e�X��x

double Zero[AXIS] = { 0.0 } ; // �s�V�q


//===============================================================

void _stdcall Timer_GravityCompensation( TMRINT *pstINTSource ) ;


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

	//---------------------- �u�@�y�{ ----------------------

	int sw = 0 ;

	while ( sw != 0x1b ) 
	{
		
		//-------------------- ��ʵ����{�� --------------------

		if ( _kbhit() )
		{

			sw = _getch() ; // ��ESC�����{��

		}

		//-------------------- �t�׫O�@�{�� --------------------

		if ( SafetyFlag == 1 )
		{

			break ; // �����{��

		}

		//---------------------- �s���-------------------------
		if (sw == 0x20) //�ť�Ų
		{
			SaveData_Data(Pos, Vel, TorN);
			std::cout<<++cnt2<<"\n";
			sw = 0;
		
		}

		//---------------------- �i�J���_ ----------------------

		MotionCard_ChangeTimer( Timer_GravityCompensation ) ;

	}

	//-------------------- �����U���\�� --------------------

	MotionCard_ServoOff() ; // Servo Off
	MotionCard_CloseCard() ; // �����b�d
	SaveData_CloseFile() ; // ������������

}


//=========================== ���O���v ===========================

void _stdcall Timer_GravityCompensation( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@


	//--------------------------- ���� ---------------------------

	RobotModel_Feedforward( Pos , Zero , Zero , TorM , TorN , TorF , TorFF ) ;

	MotionCard_DAC( TorN ) ; // �e�X����R�O


	//----------------------- �x�s������ -----------------------

	//SaveData_Data( Pos , Vel , TorN ) ;

}