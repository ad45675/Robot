#include "SaveData.h"
#include "MotionCard.h"
#include "Initialization.h"
#include "Toolbox.h"
#include "Protection.h"
#include "Control.h"
#include "PTP.h"
#include "ReadData.h"
#include "Nozzle.h"
#include "RobotModel.h"
#include "MotionCard.h"
#include "Header.h"
#include <iostream>
#include <conio.h>
#include <fstream> 
using namespace std;

#define AXIS 6
#define PI 3.1415926


//======================= Counter & Flag =======================

int Comport = 4 ; // USB comport

int SafetyFlag = 0 ; // �O�@�κX��
int EndFlag1 = 0 ; // �R�O�����X��1
int EndFlag2 = 0 ; // �R�O�����X��2
int StayCount = 0 ; // �w�ĥέp��
int StayCount2 = 0; //�l�L�έp��
int StayCount3 = 0;
int StayCount4 = 0;
int Suction_flag = 0; //�l�L���A


//===============================================================

double InitialPos[AXIS] ; // ��l����
double TargetPos[AXIS] = { -0.7854 , 0.7854 , 0.7854 , 0.0 , 0.0 , 0.0 } ; // �ؼШ���
double HomePos[AXIS]={ 0.0 , 0.524 , -0.349 , 0.0 , -0.785 , 0.0 } ;					// Home �I


// Home�I����: { 0.0 , 0.7854 , -0.7854 , 0.0 , -0.7854 , 0.0 } 
// ���եΨ���: { -0.7854 , 0.0 , 0.0 , -0.7854 , 0.0 , -0.7854 }

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

int close = 0;
double Initial[AXIS] = {0.0, 0.0, 0.0, 0.0, -1.57, 0.0}; 
double Goal_Joint[AXIS];
double Goal_Joint2[AXIS] = {0.0543818, -0.45533, 0.4199533, 0.0 , -1.5354, 0.05438}; //PTP��_�Ӥ@�I�I
double Goal_Joint3[AXIS] = {-0.95142, -0.11216, -0.397, 0.0 , -1.0167, -0.093 }; //PTP���m�Ϥ� 1 
double Goal_Joint4[AXIS] = {-0.61177, -0.52385, 0.02721, 0.0 , -1.01666, -0.093}; //PTP���m�ϥ~ 2

int Singularplace = {0.0};
float ADCVoltage[8] = { 0.0 };
int ADC_port = 6;

int TimerFlag = 0 ; // ************************  ���_�����X�� ************************

//===============================================================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource ) ;
void _stdcall Nozzle_Suck(TMRINT *pst);
void _stdcall Nozzle_Release(TMRINT *pst);
void _stdcall Timer_Stay( TMRINT *pstINTSource ) ;

//===============================================================

int main()
{

	//---------------------- Ū����� ----------------------

	//SaveData_CreateFile( "MeasuredData.txt" ) ; // �إ߹�������
	RobotModel_Beta( "Beta.txt" ) ; // ���J�t�ΰѼ�

	data(Goal_Joint, Singularplace); cout << "Goal_Joint" << endl;

	for (int i = 0; i < AXIS; i++)
	{
		cout << Goal_Joint[i] << "\t";
		if ( i == AXIS - 1)
			cout << endl;
	}

	if (Singularplace == 1)
	{

		cout << "This Place is Singular\n";
		system("Pause");

	}
	else if (Singularplace == 0)
	{

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
				close = sw;// �� �ťի� ��} �ť��� 0x20

			}

			//-------------------- �t�׫O�@�{�� --------------------

			//if ( SafetyFlag == 1 )
			//{

			//	break ; // �����{��

			//}

			//---------------------- �������_ ----------------------
			if ( TimerFlag == 0 )
			{

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��1

			}
			else if ( TimerFlag == 1 )
			{

				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step1. PTP��ؼ��I

			}
			else if ( TimerFlag == 2 )
			{

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��2

			}
			else if ( TimerFlag == 3 )
			{

				MotionCard_ChangeTimer( Nozzle_Suck ) ; // Step2. �l�L�l

			}
			else if ( TimerFlag == 4 )
			{

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��3

			}
			else if ( TimerFlag == 5 )
			{

				for ( int i = 0 ; i < AXIS ; i++ )
				{
					Goal_Joint[i] = Goal_Joint2[i] ; 
				}
				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step3. PTP��_�Ӥ@�I�I

			}
			else if(TimerFlag == 6)
			{

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��4


			}
			else if(TimerFlag == 7)
			{


				for ( int i = 0 ; i < AXIS ; i++ )
				{
					Goal_Joint[i] = Goal_Joint4[i] ; 
				}
				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step4. PTP���m��

			}
			else if(TimerFlag == 8)
			{

				if (Suction_flag == 1)
				{
					Nozzle_Release();
					Suction_flag = 0;
				}

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��5

			}
			else if(TimerFlag == 9)
			{

				MotionCard_ChangeTimer( Timer_PTPtoInitialPos ) ; // Step5. PTP��home

			}
			else if(TimerFlag == 10)
			{

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��6

			}
			else if(TimerFlag == 11)
			{

				break;

			}
		}

		//-------------------- �����U���\�� --------------------

		MotionCard_ServoOff() ; // Servo Off
		MotionCard_CloseCard() ; // �����b�d
		//SaveData_CloseFile() ; // ������������

	}
}


//====================== Step1. PTP��ؼ��I ======================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	//------------------------ �I���I���� ------------------------


	PTP_Scurve( InitialPos , Goal_Joint , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O


	//------------------------- �������_ -------------------------

	if ( EndFlag1 == 1 &&  TimerFlag == 1)
	{

		TimerFlag = 2 ; 

	}
	else if ( EndFlag1 == 1 && TimerFlag == 5)
	{

		TimerFlag = 6 ; 
	}

	else if ( EndFlag1 == 1 && TimerFlag == 7)
	{

		TimerFlag = 8 ; 
	}


	//----------------------- �x�s������ -----------------------
	

	//if (  TimerFlag == 1 || TimerFlag == 5 || TimerFlag == 7 )
	//{

		//SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl);

	//}


}


//====================== Step2. �l�L�l ======================

void _stdcall Nozzle_Suck(TMRINT *pst)
{
	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)


	//--------------------------- ���� ---------------------------

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O

	if (TimerFlag == 3 && StayCount2 < 3000)
	{
		Nozzle_Suck();
		Suction_flag = 1;
		MotionCard_ADC(ADCVoltage);
	}
	else
	{
		Nozzle_Release();
		Suction_flag = 0;
		TimerFlag = 8; // �w��5�� -> PTP�^���I
	}


	if (ADCVoltage[ADC_port] > 2 )
	{
		StayCount2--; StayCount3 ++ ;
		if (StayCount3 > 1000)
			TimerFlag = 4; //�w��3
	}
	StayCount2++;

}

//======================= Step3. PTP�^���I =======================

void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)


	//------------------------ �I���I���� ------------------------

	PTP_Scurve( InitialPos , HomePos , PosCmd , VelCmd , AccCmd , EndFlag2 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O


	//------------------------- �������_ -------------------------

	if ( EndFlag2 == 1 )
	{

		TimerFlag = 10 ;

	}

	//----------------------- �x�s������ -----------------------

	
	//if (  TimerFlag == 9 )
	//{

		//SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl) ;

	//}
	

}


//=========================== �w�ĵ{�� ===========================

void _stdcall Timer_Stay( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)


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

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // ��l����(Step3.PTP�ﰪ�@�I�I)

		}

	}
	else if (  TimerFlag == 6 && StayCount >= 4000 ) // �w��4
	{


		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // �{�b����(Step4.PTP���m��)

		}

		TimerFlag = 7 ;

	}
	else if (  TimerFlag == 8 && StayCount >= 5000 ) // �w��5
	{

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // �{�b����(Step5.PTP��home)

		}
		TimerFlag = 9 ;

	}
	else if (  TimerFlag == 10 && StayCount >= 6000 ) // �w��6
	{

		TimerFlag = 11 ;

	}


	StayCount ++ ;

}


void _stdcall Nozzle_Release(TMRINT *pst)
{

	if (StayCount4 < 2000)
	{
		Nozzle_Release();
	}

	StayCount4 ++;

}
