#include "SaveData.h"
#include "MotionCard.h"
#include "Initialization.h"
#include "Toolbox.h"
#include "Protection.h"
#include "Control.h"
#include "PTP.h"
#include "Kinematics.h"
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
#define Sol_Num 8
Kinematics MyRobot;


//======================= Counter & Flag =======================

int Comport = 4 ; // USB comport

int SafetyFlag = 0 ; // �O�@�κX��
int TimerFlag = 0 ; // ���_�����X��
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


double Target_Joint[Sol_Num][AXIS] = {0.0};
double Initial[AXIS] = {0.0, 0.0 ,0.0 ,0.0 ,-1.57, 0.0}; // ��l����
int Singularplace[Sol_Num] = {0.0};
double Goal_Joint[AXIS] ;
double Goal_Joint2[AXIS] = {-0.01142, -0.5672, 0.0364, 0.0 , -1.0399, -0.01142}; //PTP��_�Ӥ@�I�I
double Goal_Joint3[AXIS] = {-0.318197, -1.37907, 0.59069, 0.0 , -0.7824, -0.318197}; //PTP���m��
bool Singular = FALSE;

float ADCVoltage[8] = { 0.0 };
int ADC_port = 6;
Mat Solution = Mat::zeros(1,6,CV_64FC1);
//===============================================================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource ) ;
void _stdcall Nozzle_Suck(TMRINT *pst);
void _stdcall Nozzle_Release(TMRINT *pst);
void _stdcall Timer_Stay( TMRINT *pstINTSource ) ;
void MatToArray(Mat matrix ,double(&ArrayPos)[AXIS]);


//===============================================================

int main()
{

	//---------------------- Ū����� ----------------------

	SaveData_CreateFile( "MeasuredData.txt" ) ; // �إ߹�������
	RobotModel_Beta( "Beta.txt" ) ; // ���J�t�ΰѼ�

	//-------Ū��� ��̨θ� Matrix �� Array------- //
	data(Target_Joint,Singularplace);
	Solution = MyRobot.Solution(Target_Joint,InitialPos,Singularplace, Singular);
	if (Singular)
	{
		cout<<"Singular"<<endl;
		system("PAUSE");
	}
	else
	{
		MatToArray(Solution ,Goal_Joint);
		cout<<"Goal_Joint"<<Solution<< "123"<<Singular <<endl;
		//-------------------- �}�ҦU���\�� --------------------

		printf( "Press any key to start !\n" ) ; 
		printf( "Press 'Esc' to end program.\n" ) ;

		while ( _kbhit() == 0 ) ; // �����N��}�l�{��

		MotionCard_OpenCard( Comport ) ; // �}�Ҷb�d
		Initialization( PosCmd ) ; // ����l���שR�O & ��sLSF
		MotionCard_ServoOn() ; // Servo On

		//---------------------- �u�@�y�{ ----------------------

		int sw = 0 ;
		int close = 0;

		while ( sw != 0x1b  ) 
		{
			if (Singular)
				break;

			//-------------------- ��ʵ����{�� --------------------

			if ( _kbhit() )
			{

				sw = _getch() ; // ��ESC�����{��
				close = sw;// �� �ťի� ��}

			}

			//-------------------- �t�׫O�@�{�� --------------------

			//if ( SafetyFlag == 1 )
			//{

			//	break ; // �����{��

			//}

			//---------------------- �������_ ----------------------

			if ( TimerFlag == 0 )
			{
				printf("TimerFlag == 0 \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��1


			}
			else if ( TimerFlag == 1 )
			{
				printf("TimerFlag == 1, ptp��ؼ��I \r");

				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step1. PTP��ؼ��I
			
			}
			else if ( TimerFlag == 2 )
			{
				printf("TimerFlag == 2, �w��2  \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��2

			}
			else if ( TimerFlag == 3 )
			{
				printf("TimerFlag == 3, �l�L�l \r");
				MotionCard_ChangeTimer( Nozzle_Suck ) ; // Step2. �l�L�l

			}
			else if ( TimerFlag == 4 )
			{
				printf("TimerFlag == 4 , �w��3 \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��3

			}
			else if ( TimerFlag == 5 )
			{
				printf("TimerFlag == 5 , PTP��_�Ӥ@�I�I\r");
			
				for ( int i = 0 ; i < AXIS ; i++ )
				{
					Goal_Joint[i] = Goal_Joint2[i] ; 
				}
		

				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step3. PTP��_�Ӥ@�I�I

			}
			else if(TimerFlag == 6)
			{
				printf("TimerFlag == 6 , �w��4\r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��4


			}
			else if(TimerFlag == 7)
			{
				printf("TimerFlag == 7 ,  PTP���m��\r");
				for ( int i = 0 ; i < AXIS ; i++ )
				{

					Goal_Joint[i] = Goal_Joint3[i] ; 

				}
				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step4. PTP���m��

			}
			else if(TimerFlag == 8)
			{
			
				printf("TimerFlag == 8 ,  �w��5 ����\r");
				if (Suction_flag == 1)
				{
					Nozzle_Release();
					Suction_flag = 0;

				}
		
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��5

			}
			else if(TimerFlag == 9)
			{
				printf("TimerFlag == 9 ,  PTP��home\r");
				MotionCard_ChangeTimer( Timer_PTPtoInitialPos ) ; // Step5. PTP��home
			}
			else if(TimerFlag == 10)
			{
				printf("TimerFlag == 10 , �w��6\r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��6
			}
			else if(TimerFlag == 11)
			{
				//MotionCard_ChangeTimer(Nozzle_Release);
				break;
			}
			else if(TimerFlag == 12)
			{
				break;
			}


		}

		//-------------------- �����U���\�� --------------------

		MotionCard_ServoOff() ; // Servo Off
		MotionCard_CloseCard() ; // �����b�d
		SaveData_CloseFile() ; // ������������
	}


}


//====================== Step1. PTP��ؼ��I ======================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource )
{

	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@

	//------------------------ �I���I���� ------------------------
	

	PTP_Scurve( InitialPos , Goal_Joint , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	//MotionCard_DAC( TorCtrl ) ; // �e�X����R�O

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//------------------------- �������_ -------------------------

	if ( EndFlag1 == 1 &&  TimerFlag == 1)
	{

		TimerFlag = 2 ; 

	}
	else if ( EndFlag1 == 1 && TimerFlag == 5)
	{
	
		TimerFlag =6 ; 
	}

	else if ( EndFlag1 == 1 && TimerFlag == 7)
	{
	
		TimerFlag =8 ; 
	}


	//----------------------- �x�s������ -----------------------

	if (  TimerFlag == 1 || TimerFlag == 5 || TimerFlag == 7 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}

}


//====================== Step2. �l�L�l ======================
void _stdcall Nozzle_Suck(TMRINT *pst)

{
	//---------------------- Ū�����סB�t�� ----------------------

	MotionCard_Encoder( Pos ) ; // ���c����(rad)
	Toolbox_LSF( Pos , Vel ) ; // ���c�t��(rad/s)

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@


	//--------------------------- ���� ---------------------------

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O

	if (TimerFlag == 3 && StayCount2 < 5000)
	{
		Nozzle_Suck();
		Suction_flag = 1;
		MotionCard_ADC(ADCVoltage);
	}
	else
	{
		Nozzle_Release();
		Suction_flag = 0;
		TimerFlag = 4; // �w��3�� -> PTP�^���I
	}


	 
	if (ADCVoltage[ADC_port] > 2 )
	{
		printf("�l��F\n");
		StayCount2--;StayCount3 ++ ;
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

	Protection( Vel , SafetyFlag ) ; // �t�׫O�@


	//------------------------ �I���I���� ------------------------

	PTP_Scurve( InitialPos , HomePos , PosCmd , VelCmd , AccCmd , EndFlag2 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//----------------------- �x�s������ -----------------------

	if (  TimerFlag == 9 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}


	//------------------------- �������_ -------------------------

	if ( EndFlag2 == 1 )
	{

		TimerFlag = 10 ;

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

void MatToArray(Mat matrix ,double(&ArrayPos)[AXIS])
{

	ArrayPos[0] = matrix.at<double>(0,0);
	ArrayPos[1] = matrix.at<double>(0,1);
	ArrayPos[2] = matrix.at<double>(0,2);
	ArrayPos[3] = matrix.at<double>(0,3);
	ArrayPos[4] = matrix.at<double>(0,4);
	ArrayPos[5] = matrix.at<double>(0,5);

	
}

void _stdcall Nozzle_Release(TMRINT *pst)
{
	if (StayCount4 < 2000)
	{
		cout << "Release" <<endl;
		Nozzle_Release();
	}

	StayCount4 ++;
}