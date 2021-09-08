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

int SafetyFlag = 0 ; // 保護用旗標
int TimerFlag = 0 ; // 中斷切換旗標
int EndFlag1 = 0 ; // 命令結束旗標1
int EndFlag2 = 0 ; // 命令結束旗標2
int StayCount = 0 ; // 緩衝用計數


//===============================================================

double InitialPos[AXIS] ; // 初始角度
double TargetPos[AXIS] = { 0.0 , 0.7854 , -0.7854 , 0.0 , -0.7854 , 0.0 } ; // 目標角度
// Home點角度: { 0.0 , 0.7854 , -0.7854 , 0.0 , -0.7854 , 0.0 } 

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


//===============================================================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource ) ;
void _stdcall Timer_Stay( TMRINT *pstINTSource ) ;


//===============================================================

int main()
{

	//---------------------- 讀取資料 ----------------------

	SaveData_CreateFile( "MeasuredData.txt" ) ; // 建立實驗資料檔
	RobotModel_Beta( "Beta.txt" ) ; // 載入系統參數

	//-------------------- 開啟各項功能 --------------------

	printf( "Press any key to start !\n" ) ; 
	printf( "Press 'Esc' to end program.\n" ) ;

	while ( _kbhit() == 0 ) ; // 按任意鍵開始程式
	
	MotionCard_OpenCard( Comport ) ; // 開啟軸卡
	Initialization( PosCmd ) ; // 給初始角度命令 & 更新LSF
	MotionCard_ServoOn() ; // Servo On

	
	//-------------------- 關閉各項功能 --------------------

	MotionCard_ServoOff() ; // Servo Off
	MotionCard_CloseCard() ; // 關閉軸卡
	SaveData_CloseFile() ; // 關閉實驗資料檔

}


//====================== Step1. PTP到目標點 ======================

void _stdcall Timer_PTPtoTargetPos( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	Protection( Vel , SafetyFlag ) ; // 速度保護


	//------------------------ 點對點控制 ------------------------

	PTP_Scurve( InitialPos , TargetPos , PosCmd , VelCmd , AccCmd , EndFlag1 ) ; // 點對點命令

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	MotionCard_DAC( TorCtrl ) ; // 送出控制命令

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//------------------------- 切換中斷 -------------------------

	if ( EndFlag1 == 1 )
	{

		TimerFlag = 2 ;

	}


	//----------------------- 儲存實驗資料 -----------------------

	if (  TimerFlag == 1 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}

}


//======================= Step2. PTP回原點 =======================

void _stdcall Timer_PTPtoInitialPos( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	Protection( Vel , SafetyFlag ) ; // 速度保護


	//------------------------ 點對點控制 ------------------------

	PTP_Scurve( TargetPos , InitialPos , PosCmd , VelCmd , AccCmd , EndFlag2 ) ; // 點對點命令

	Control_Feedback( Pos , Vel , PosCmd , TorCtrl ) ; // 回授控制

	MotionCard_DAC( TorCtrl ) ; // 送出控制命令

	RobotModel_Feedforward( Pos , Vel , AccCmd , TorM , TorN , TorF , TorFF ) ;
	Toolbox_Filter( TorF , TorF_f ) ;


	//----------------------- 儲存實驗資料 -----------------------

	if (  TimerFlag == 3 )
	{

		SaveData_Data( Pos , Vel , PosCmd , VelCmd , TorCtrl , TorM , TorN , TorF_f ) ;

	}


	//------------------------- 切換中斷 -------------------------

	if ( EndFlag2 == 1 )
	{

		TimerFlag = 4 ;

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

	}

	StayCount ++ ;

}
