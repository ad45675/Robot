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

double Goal_Joint2[AXIS] = {-0.01142, -0.5672, 0.0364, 0.0 , -1.0399, -0.01142}; //PTP��_�Ӥ@�I�I
double Goal_Joint3[AXIS] = {-0.71737, -0.81042, 0.4166, 0.0 , -1.17699, -0.7173}; //PTP���m��


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


//double Target_Joint[Sol_Num][AXIS] = {0.0};
double Goal_Joint[AXIS] ;


bool Singular = FALSE;
int Singularplace = {0.0};
float ADCVoltage[8] = { 0.0 };
int ADC_port = 6;
//Mat Solution = Mat::zeros(1,6,CV_64FC1);

//===============================================================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_Stay( TMRINT *pstINTSource ) ;
void MatToArray(Mat matrix ,double(&ArrayPos)[AXIS]);


//===============================================================

int main()
{

	//---------------------- Ū����� ----------------------

	SaveData_CreateFile( "MeasuredData.txt" ) ; // �إ߹�������
	RobotModel_Beta( "Beta.txt" ) ; // ���J�t�ΰѼ�


	//-------Ū��� ��̨θ� Matrix �� Array------- //
	data(Goal_Joint,Singularplace); cout << "Goal_Joint"<< endl;
	for (int i=0;i<AXIS;i++)
	{
		cout <<Goal_Joint[i]<<" ";
	}
	
	if (Singularplace == 1)
	{
		cout<<"Singular"<<endl;
		system("PAUSE");
	}
	else
	{
		//MatToArray(Solution ,Goal_Joint);
		//cout<<"Goal_Joint"<<Solution<<endl;

		//-------Ū��� ��̨θ� Matrix �� Array------- //


		//-------------------- �}�ҦU���\�� --------------------

		printf( "\nPress any key to start !\n" ) ; 
		printf( "Press 'Esc' to end program.\n" ) ;

		while ( _kbhit() == 0 ) ; // �����N��}�l�{��
	
		MotionCard_OpenCard( Comport ) ; // �}�Ҷb�d
		Initialization( PosCmd ) ; // ����l���שR�O & ��sLSF
		MotionCard_ServoOn() ; // Servo On

		//---------------------- �u�@�y�{ ----------------------

		int sw = 0 ;
		int close = 0;

		while ( sw != 0x1b ) 
		{
		
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

				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��1

			}
			else if ( TimerFlag == 1 )
			{
				//printf("TimerFlag == 1, ptp��ؼ��I \r");
				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step1. PTP��ؼ��I
			
			}
			else if ( TimerFlag == 2 )
			{
				//printf("TimerFlag == 2, �w��2  \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // �w��2

			}
			else if ( TimerFlag == 3 )
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

	//------------------------ �I���I���� ------------------------
	

	PTP_Scurve( InitialPos , Goal_Joint , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // �I���I�R�O

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // �^�±���

	MotionCard_DAC( TorCtrl ) ; // �e�X����R�O


	//------------------------- �������_ -------------------------

	if ( EndFlag1 == 1 &&  TimerFlag == 1)
	{

		TimerFlag = 2 ; 

	}



	//----------------------- �x�s������ -----------------------

	if (  TimerFlag == 1 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl);

	}

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

