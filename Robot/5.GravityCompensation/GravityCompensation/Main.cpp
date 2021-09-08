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

int SafetyFlag = 0 ; // 保護用旗標
int TimerFlag = 0 ; // 中斷切換旗標
int EndFlag1 = 0 ; // 命令結束旗標1
int EndFlag2 = 0 ; // 命令結束旗標2
int StayCount = 0 ; // 緩衝用計數
double cnt2 = 0 ; // 存資料計數器


//===============================================================

double Pos[AXIS] = { 0.0 } ; // 編碼器角度
double Vel[AXIS] = { 0.0 } ; // LSF速度
double PosCmd[AXIS] = { 0.0 } ; // 角度命令
double VelCmd[AXIS] = { 0.0 } ; // 速度命令
double AccCmd[AXIS] = { 0.0 } ; // 加速度命令
double TorCtrl[AXIS] = { 0.0 } ; // 轉矩控制訊號

double TorM[AXIS] = { 0.0 } ; // 前饋慣性力
double TorN[AXIS] = { 0.0 } ; // 前饋科氏力&重力
double TorF[AXIS] = { 0.0 } ; // 前饋摩擦力
double TorFF[AXIS] = { 0.0 } ; // 總前饋轉矩

double Zero[AXIS] = { 0.0 } ; // 零向量


//===============================================================

void _stdcall Timer_GravityCompensation( TMRINT *pstINTSource ) ;


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

	//---------------------- 工作流程 ----------------------

	int sw = 0 ;

	while ( sw != 0x1b ) 
	{
		
		//-------------------- 手動結束程式 --------------------

		if ( _kbhit() )
		{

			sw = _getch() ; // 按ESC結束程式

		}

		//-------------------- 速度保護程式 --------------------

		if ( SafetyFlag == 1 )
		{

			break ; // 結束程式

		}

		//---------------------- 存資料-------------------------
		if (sw == 0x20) //空白鑑
		{
			SaveData_Data(Pos, Vel, TorN);
			std::cout<<++cnt2<<"\n";
			sw = 0;
		
		}

		//---------------------- 進入中斷 ----------------------

		MotionCard_ChangeTimer( Timer_GravityCompensation ) ;

	}

	//-------------------- 關閉各項功能 --------------------

	MotionCard_ServoOff() ; // Servo Off
	MotionCard_CloseCard() ; // 關閉軸卡
	SaveData_CloseFile() ; // 關閉實驗資料檔

}


//=========================== 重力補償 ===========================

void _stdcall Timer_GravityCompensation( TMRINT *pstINTSource )
{

	//---------------------- 讀取角度、速度 ----------------------

	MotionCard_Encoder( Pos ) ; // 機構角度(rad)
	Toolbox_LSF( Pos , Vel ) ; // 機構速度(rad/s)

	Protection( Vel , SafetyFlag ) ; // 速度保護


	//--------------------------- 控制 ---------------------------

	RobotModel_Feedforward( Pos , Zero , Zero , TorM , TorN , TorF , TorFF ) ;

	MotionCard_DAC( TorN ) ; // 送出控制命令


	//----------------------- 儲存實驗資料 -----------------------

	//SaveData_Data( Pos , Vel , TorN ) ;

}