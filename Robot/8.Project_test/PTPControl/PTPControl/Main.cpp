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


double Target_Joint[Sol_Num][AXIS] = {0.0};
double Initial[AXIS] = {0.0, 0.0 ,0.0 ,0.0 ,-1.57, 0.0}; // 初始角度
int Singularplace[Sol_Num] = {0.0};
double Goal_Joint[AXIS] ;
double Goal_Joint2[AXIS] = {-0.01142, -0.5672, 0.0364, 0.0 , -1.0399, -0.01142}; //PTP抬起來一點點
double Goal_Joint3[AXIS] = {-0.318197, -1.37907, 0.59069, 0.0 , -0.7824, -0.318197}; //PTP到放置區
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

	//---------------------- 讀取資料 ----------------------

	SaveData_CreateFile( "MeasuredData.txt" ) ; // 建立實驗資料檔
	RobotModel_Beta( "Beta.txt" ) ; // 載入系統參數

	//-------讀資料 找最佳解 Matrix 轉 Array------- //
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
		//-------------------- 開啟各項功能 --------------------

		printf( "Press any key to start !\n" ) ; 
		printf( "Press 'Esc' to end program.\n" ) ;

		while ( _kbhit() == 0 ) ; // 按任意鍵開始程式

		MotionCard_OpenCard( Comport ) ; // 開啟軸卡
		Initialization( PosCmd ) ; // 給初始角度命令 & 更新LSF
		MotionCard_ServoOn() ; // Servo On

		//---------------------- 工作流程 ----------------------

		int sw = 0 ;
		int close = 0;

		while ( sw != 0x1b  ) 
		{
			if (Singular)
				break;

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
				printf("TimerFlag == 0 \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝1


			}
			else if ( TimerFlag == 1 )
			{
				printf("TimerFlag == 1, ptp到目標點 \r");

				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step1. PTP到目標點
			
			}
			else if ( TimerFlag == 2 )
			{
				printf("TimerFlag == 2, 緩衝2  \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝2

			}
			else if ( TimerFlag == 3 )
			{
				printf("TimerFlag == 3, 吸嘴吸 \r");
				MotionCard_ChangeTimer( Nozzle_Suck ) ; // Step2. 吸嘴吸

			}
			else if ( TimerFlag == 4 )
			{
				printf("TimerFlag == 4 , 緩衝3 \r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝3

			}
			else if ( TimerFlag == 5 )
			{
				printf("TimerFlag == 5 , PTP抬起來一點點\r");
			
				for ( int i = 0 ; i < AXIS ; i++ )
				{
					Goal_Joint[i] = Goal_Joint2[i] ; 
				}
		

				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step3. PTP抬起來一點點

			}
			else if(TimerFlag == 6)
			{
				printf("TimerFlag == 6 , 緩衝4\r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝4


			}
			else if(TimerFlag == 7)
			{
				printf("TimerFlag == 7 ,  PTP到放置區\r");
				for ( int i = 0 ; i < AXIS ; i++ )
				{

					Goal_Joint[i] = Goal_Joint3[i] ; 

				}
				MotionCard_ChangeTimer( Timer_PTPtoTargetPos ) ; // Step4. PTP到放置區

			}
			else if(TimerFlag == 8)
			{
			
				printf("TimerFlag == 8 ,  緩衝5 放物體\r");
				if (Suction_flag == 1)
				{
					Nozzle_Release();
					Suction_flag = 0;

				}
		
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝5

			}
			else if(TimerFlag == 9)
			{
				printf("TimerFlag == 9 ,  PTP到home\r");
				MotionCard_ChangeTimer( Timer_PTPtoInitialPos ) ; // Step5. PTP到home
			}
			else if(TimerFlag == 10)
			{
				printf("TimerFlag == 10 , 緩衝6\r");
				MotionCard_ChangeTimer( Timer_Stay ) ; // 緩衝6
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

	Protection( Vel , SafetyFlag ) ; // 速度保護

	//------------------------ 點對點控制 ------------------------
	

	PTP_Scurve( InitialPos , Goal_Joint , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // 點對點命令

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	//MotionCard_DAC( TorCtrl ) ; // 送出控制命令

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//------------------------- 切換中斷 -------------------------

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


	//----------------------- 儲存實驗資料 -----------------------

	if (  TimerFlag == 1 || TimerFlag == 5 || TimerFlag == 7 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}

}


//====================== Step2. 吸嘴吸 ======================
void _stdcall Nozzle_Suck(TMRINT *pst)

{
	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	Protection( Vel , SafetyFlag ) ; // 速度保護


	//--------------------------- 控制 ---------------------------

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	MotionCard_DAC( TorCtrl ) ; // 送出控制命令

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
		TimerFlag = 4; // 緩衝3後 -> PTP回原點
	}


	 
	if (ADCVoltage[ADC_port] > 2 )
	{
		printf("吸住了\n");
		StayCount2--;StayCount3 ++ ;
		if (StayCount3 > 1000)
			TimerFlag = 4; //緩衝3
	}
	
	StayCount2++;
	
}

//======================= Step3. PTP回原點 =======================

void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	Protection( Vel , SafetyFlag ) ; // 速度保護


	//------------------------ 點對點控制 ------------------------

	PTP_Scurve( InitialPos , HomePos , PosCmd , VelCmd , AccCmd , EndFlag2 ) ; // 點對點命令

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	MotionCard_DAC( TorCtrl ) ; // 送出控制命令

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//----------------------- 儲存實驗資料 -----------------------

	if (  TimerFlag == 9 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}


	//------------------------- 切換中斷 -------------------------

	if ( EndFlag2 == 1 )
	{

		TimerFlag = 10 ;

	}

}


//=========================== 緩衝程式 ===========================

void _stdcall Timer_Stay( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	Protection( Vel , SafetyFlag ) ; // 速度保護


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
	else if (  TimerFlag == 4 && StayCount >= 3000 ) // 緩衝3
	{

		TimerFlag = 5 ;

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // 初始角度(Step3.PTP抬高一點點)

		}


	}
	else if (  TimerFlag == 6 && StayCount >= 4000 ) // 緩衝4
	{
		

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // 現在角度(Step4.PTP到放置區)

		}

		TimerFlag = 7 ;

	}
	else if (  TimerFlag == 8 && StayCount >= 5000 ) // 緩衝5
	{

		for ( int i = 0 ; i < AXIS ; i++ )
		{

			InitialPos[i] = Pos[i] ; // 現在角度(Step5.PTP到home)

		}
		TimerFlag = 9 ;

	}
	else if (  TimerFlag == 10 && StayCount >= 6000 ) // 緩衝6
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