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

int SafetyFlag = 0 ; // 保護用旗標
int TimerFlag = 0 ; // 中斷切換旗標
int EndFlag1 = 0 ; // 命令結束旗標1
int EndFlag2 = 0 ; // 命令結束旗標2
int StayCount = 0 ; // 緩衝用計數
int StayCount2 = 0; //吸嘴用計數
int StayCount3 = 0;
int StayCount4 = 0;
int Suction_flag = 0; //吸嘴狀態


//===============================================================

double InitialPos[AXIS] ; // 初始角度
double TargetPos[AXIS] = { -0.7854 , 0.7854 , 0.7854 , 0.0 , 0.0 , 0.0 } ; // 目標角度
double HomePos[AXIS]={ 0.0 , 0.524 , -0.349 , 0.0 , -0.785 , 0.0 } ;					// Home 點

double Goal_Joint2[AXIS] = {-0.01142, -0.5672, 0.0364, 0.0 , -1.0399, -0.01142}; //PTP抬起來一點點
double Goal_Joint3[AXIS] = {-0.71737, -0.81042, 0.4166, 0.0 , -1.17699, -0.7173}; //PTP到放置區


// Home點角度: { 0.0 , 0.7854 , -0.7854 , 0.0 , -0.7854 , 0.0 } 
// 測試用角度: { -0.7854 , 0.0 , 0.0 , -0.7854 , 0.0 , -0.7854 }

double Pos[AXIS] = { 0.0 } ; // 編碼器角度
double Vel[AXIS] = { 0.0 } ; // LSF速度
double PosCmd[AXIS] = { 0.0 } ; // 角度命令
double VelCmd[AXIS] = { 0.0 } ; // 速度命令
double AccCmd[AXIS] = { 0.0 } ; // 加速度命令
double TorCtrl[AXIS] = { 0.0 } ; // 轉矩控制訊號

double TorM[AXIS] = { 0.0 } ; // 前饋慣性力
double TorN[AXIS] = { 0.0 } ; // 前饋科氏力&重力
double TorF[AXIS] = { 0.0 } ; // 前饋摩擦力
double TorF_f[AXIS] = { 0.0 } ; // 濾波後前饋摩擦力
double TorFF[AXIS] = { 0.0 } ; // 總前饋轉矩


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

	//---------------------- 讀取資料 ----------------------

	SaveData_CreateFile( "MeasuredData.txt" ) ; // 建立實驗資料檔
	RobotModel_Beta( "Beta.txt" ) ; // 載入系統參數


	//-------讀資料 找最佳解 Matrix 轉 Array------- //
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

		//-------讀資料 找最佳解 Matrix 轉 Array------- //


		//-------------------- 開啟各項功能 --------------------

		printf( "\nPress any key to start !\n" ) ; 
		printf( "Press 'Esc' to end program.\n" ) ;

		while ( _kbhit() == 0 ) ; // 按任意鍵開始程式
	
		MotionCard_OpenCard( Comport ) ; // 開啟軸卡
		Initialization( PosCmd ) ; // 給初始角度命令 & 更新LSF
		MotionCard_ServoOn() ; // Servo On

		//---------------------- 工作流程 ----------------------

		int sw = 0 ;
		int close = 0;

		while ( sw != 0x1b ) 
		{
		
			//-------------------- 手動結束程式 --------------------

			if ( _kbhit() )
			{

				sw = _getch() ; // 按ESC結束程式
				close = sw;// 按 空白建 放開

			}

			//-------------------- 速度保護程式 --------------------

			//if ( SafetyFlag == 1 )
			//{

			//	break ; // 結束程式

			//}

			//---------------------- 切換中斷 ----------------------

			if ( TimerFlag == 0 )
			{

				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝1

			}
			else if ( TimerFlag == 1 )
			{
				//printf("TimerFlag == 1, ptp到目標點 \r");
				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step1. PTP到目標點
			
			}
			else if ( TimerFlag == 2 )
			{
				//printf("TimerFlag == 2, 緩衝2  \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝2

			}
			else if ( TimerFlag == 3 )
			{
				
				break;

			}
			

		}

		//-------------------- 關閉各項功能 --------------------

		MotionCard_ServoOff() ; // Servo Off
		MotionCard_CloseCard() ; // 關閉軸卡
		SaveData_CloseFile() ; // 關閉實驗資料檔
	}
}


//====================== Step1. PTP到目標點 ======================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	//------------------------ 點對點控制 ------------------------
	

	PTP_Scurve( InitialPos , Goal_Joint , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // 點對點命令

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	MotionCard_DAC( TorCtrl ) ; // 送出控制命令


	//------------------------- 切換中斷 -------------------------

	if ( EndFlag1 == 1 &&  TimerFlag == 1)
	{

		TimerFlag = 2 ; 

	}



	//----------------------- 儲存實驗資料 -----------------------

	if (  TimerFlag == 1 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl);

	}

}



//=========================== 緩衝程式 ===========================

void _stdcall Timer_Stay( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)


	//--------------------------- 控制 ---------------------------

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	MotionCard_DAC( TorCtrl ) ; // 送出控制命令


	//------------------------- 切換中斷 -------------------------

	if (  TimerFlag == 0 && StayCount >= 1000 ) // 緩衝1
	{

		TimerFlag = 1 ;

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // 初始角度(Step1.PTP)

		}

	}
	else if (  TimerFlag == 2 && StayCount >= 2000 ) // 緩衝2
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

