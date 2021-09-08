#include "MyRobot_DrvComunicate.h" // 需在最前!
#include "MotionCard.h"


//=================== 內部用函式 ===================

void MotionCard_ABS( int Comport ) ; // 絕對型編碼器
void MotionCard_GetEnc( int axis , long (&EncPulse) ) ; // 讀編碼器pulse
void MotionCard_ClearProtectionBin() ;  // 保護程式
void MotionCard_ExamineProtect() ; // 保護程式


//==================== 開啟軸卡 ====================

void MotionCard_OpenCard( int Comport )
{

	//MotionCard_ExamineProtect() ; // 保護程式

	int nRet = IMC_OpenDevice( 0 , 0 ) ; // 軸卡初始化

	if( nRet == 0 )
	{

		printf( "Initialization Fails !" ) ;

	}
	else
	{

		IMC_GLB_ResetModule( RESET_ALL , 0 ) ; // 重置 IMP 模組
		IMC_GLB_SetInterruptMask( IMC_ALL_INT_UNMASK , 0 ) ; // UnMask Interrupt

		//----------------- 編碼器 -----------------

		for( int i = 0 ; i < AXIS ; i++ )
		{

			IMC_ENC_SetInputFormat( i , ENC_FMT_AB , 0 ) ; // 設定編碼器輸入格式為 A/B 型
			IMC_ENC_SetInputRate( i , ENC_RATE_X4 , 0 ) ; // 設定編碼器輸入解碼倍率 x4
			IMC_ENC_ClearCounter( i , 1 , 0 ) ; // 開啟清除編碼器
			IMC_ENC_ClearCounter( i , 0 , 0 ) ; // 關閉清除編碼器
			IMC_ENC_StartCounter( i , 1 , 0 ) ; // 開始編碼器輸入 

		}

		//MotionCard_ABS( Comport ) ; // 讀取絕對型編碼器(Pulse)

		//---------------- DAC & ADC ----------------

		for( int i = 0 ; i < AXIS + 2  ; i++ )
		{

			IMC_DAC_SelectSource( i , DAC_SOURCE_SOFT , 0 ) ; // 設定 DAC 命令源為軟體規劃
			IMC_DAC_EnableChannel( i , 1 , 0 ) ; // 開啟 DAC 功能
			IMC_ADC_EnableChannel( i , 1 , 0 ) ; // 開啟 ADC 功能	

		}

		IMC_ADC_SetConverterMode( 2 , 0 ) ;

		IMC_DAC_StartConverter( 1 , 0 ) ; // 開始 DAC 轉換
		IMC_ADC_StartConverter( 1 , 0 ) ; // 開始 ADC 轉換

		for( int i = 0 ; i < AXIS + 2  ; i++ )
		{

			IMC_DAC_SetOutputVoltage( i , 0.0 , 0 ) ; // DAC 電壓清空

		}

		//------------------ RIO ------------------

		IMC_RIO_SetClockDivider( RIO_SET0 , 255 , 0 ) ; //設定ClockDivider, IMP預設ClockDivider為255 ,此設定將影響傳輸速度, 且必須與所有子版設定相同
		IMC_RIO_EnableSlaveControl( RIO_SET0 , RIO_SLAVE0 , 1 , 0 ) ; // 開啟Slave傳輸功能
		IMC_RIO_EnableSetControl( RIO_SLAVE0 , 1 , 0 ) ;

		//---------------- 中斷功能 ----------------

		IMC_TMR_SetTimer( 1000 , 0 ) ; // 設定計時器計時時間 1000us (1豪秒)
		IMC_TMR_SetTimerEnable( 1 , 0 ) ; // 開啟計時器計時功能
		IMC_TMR_SetTimerIntEnable( 1, 0 ) ; // 開啟計時器中斷功能

	}

}


//==================== 關閉軸卡 ====================

void MotionCard_CloseCard()
{

	IMC_TMR_SetTimerIntEnable( 0 , 0 ) ; // 關閉計時器中斷功能
	IMC_TMR_SetTimerEnable( 0 , 0 ) ; // 關閉計時器計時功能

	for( int i = 0 ; i < AXIS  ; i++ )
	{

		IMC_DAC_SetOutputVoltage( i , 0.0 , 0 ) ; // DAC電壓清空

	}

	IMC_DAC_StartConverter( 0 , 0 ) ; // 停止 DAC 轉換
	IMC_ADC_StartConverter( 0 , 0 ) ; // 停止 ADC 轉換

	for( int i = 0 ; i < AXIS  ; i++ )
	{

		IMC_ENC_StartCounter( i , 0 , 0 ) ; // 停止編碼器輸入
		IMC_DAC_EnableChannel( i , 0 , 0 ) ; // DAC中斷功能
		IMC_ADC_EnableChannel( i , 0 , 0 ) ; // ADC中斷功能

	}

	IMC_RIO_EnableSlaveControl( RIO_SET0,RIO_SLAVE0 , 0 , 0 ) ; // 關閉Slave傳輸功能
	IMC_RIO_EnableSetControl( RIO_SLAVE0 , 0 , 0 ) ;

	IMC_CloseIfOpen( 0 ) ; // 關閉 IMP 模組

	WinExec( "taskkill /im safety_monitoring.exe", SW_HIDE ) ; // kill protection program 保護程式
	MotionCard_ClearProtectionBin() ; // 保護程式

}


//============= Servo On (Torque Mode) =============

void MotionCard_ServoOn()
{
	
	DWORD input ;
	do
	{
		IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT2 , 0x0f09 , 0 ) ; // 寫入 Servo ON
		IMC_RIO_GetInputValue( RIO_SET0 , RIO_SLAVE0 , RIO_PORT2 , &input , 0 ) ; // 讀取 確認是否 Servo ON

	}while( input && ( 0x0100 ) != 0x0100 ) ;
	printf( "servo on success\n" ) ;
	
	IMC_RIO_SetOutputValue( RIO_SET0 , RIO_SLAVE0 , RIO_PORT0 , 0x3500 , 0 ) ; // 軸 1,3,5,6 馬達反向裝
	IMC_RIO_SetOutputValue( RIO_SET0 , RIO_SLAVE0 , RIO_PORT1 , 0x000a , 0 ) ; // 軸 2,4 馬達正向裝

}


//==================== Servo Off ====================

void MotionCard_ServoOff()
{

	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT2 , 0x0000 , 0) ; // RIO_PORT2( Servo off等 ) 需優先關閉且需加 Sleep
	Sleep( 50 ) ;
	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT1 , 0x0000 , 0) ; // RIO_PORT1( ABSR + BRK )
	Sleep( 50 ) ;
	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT0 , 0x0000 , 0) ; // RIO_PORT0( Reset + ABSM )
	Sleep( 50 ) ;
	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT3 , 0x0000 , 0) ; // RIO_PORT3( 氣壓閥 + 其他 )
	Sleep( 50 ) ;

}


//================= 讀取機構角度值 =================

double ABSPulse[AXIS] ;

void MotionCard_ABS( int Comport ) // 馬達絕對型編碼器(讀完後固定值)
{

	static double DirectionABS[AXIS] = { -1 , 1 , -1 , 1 , -1 , -1 } ;
	MyRobot_DrvComunicate* drv_talk = new MyRobot_DrvComunicate( Comport , 9600 ) ;

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		int ReadPulse = 0 ;
		drv_talk->MyRobot_ABS_Position( i , ReadPulse ) ;
		ABSPulse[i] = DirectionABS[i] * ReadPulse ;  // 絕對型編碼器值(Pulse)

	}
	
	delete drv_talk ;

}

void MotionCard_GetEnc( int axis , long (&MotorEncPulse) ) // 馬達增量+絕對編碼器值
{

	long lCount ;

	IMC_ENC_ReadCounter( axis, &lCount, 0 ) ; // 讀取增量值
	MotorEncPulse = lCount + (long)ABSPulse[axis] ; // 增量+絕對編碼器值(Pulse)

}

void MotionCard_Encoder( double (& RobotPosRad)[AXIS] ) // 機構角度值(rad)
{

	const double CircleRad = 2 * PI ;
	long MotorEncPulse[AXIS] = { 0 } ;

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		MotionCard_GetEnc( i , MotorEncPulse[i] ) ; // 讀取增量+絕對值
		RobotPosRad[i] = ( (double)MotorEncPulse[i] / 3600000.0 ) * CircleRad ; // 機構角度(Rad)，機構一圈3600000個Pulse(以驅動器參數調整)

	}

}


//====================== DAC ======================

const double GearRatio[AXIS] = { 120.0 , 120.0 , 120.0 , 102.0 , 80.0 , 51.0 } ; // 減速比
const double RatedTorque[AXIS] = { 1.3 , 1.3 , 1.3 , 0.32 , 0.32 , 0.32 } ; // 馬達額定轉矩
//const double MaximumTorque[AXIS] = { 3.8 , 3.8 , 3.8 , 0.95 , 0.95 , 0.95 } ; // 馬達最大轉矩
const double MaxVoltage = 8 ; // 最大電壓

void MotionCard_DAC( double RobotTorCtrl[AXIS] )
{

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		double MotorTorCtrl = RobotTorCtrl[i] / GearRatio[i] ; // 馬達轉矩控制命令(Nm)

		if ( MotorTorCtrl > RatedTorque[i] )
		{
			MotorTorCtrl = RatedTorque[i] ; // 馬達轉矩限制
		}

		if ( MotorTorCtrl < -RatedTorque[i] )
		{
			MotorTorCtrl = -RatedTorque[i] ; // 馬達轉矩限制
		}

		double ControlSignal = ( MotorTorCtrl / RatedTorque[i] ) * MaxVoltage ; // 控制訊號(V): 最大電壓對應(額定或最大)轉矩(視TLC設定)
		
		IMC_DAC_SetOutputVoltage( i , (float)ControlSignal , 0 ) ;

	}

}


//====================== ADC ======================

void MotionCard_ADC( float (&ADCVoltage)[8] )
{

	//float ADCVoltage[AXIS] ;

	for ( int i = 0 ; i < 8 ; i++ )
	{

		IMC_ADC_GetInputVoltage( i , &ADCVoltage[i] , 0 ) ; // 讀取ADC電壓
		//ADCVoltage[i] = ADCVoltage[i] + ( 0.03482 * ADCVoltage[i] - 0.016577 ) ; // ADC偏置補償 (Offset)

	}

}


//=================== 切換中斷 ===================

void MotionCard_ChangeTimer( TMRISR Timer )
{

	IMC_TMR_SetISRFunction( Timer , 0 ) ; //串接中斷服務函式

}


//=================== 保護程式 ===================

void MotionCard_ClearProtectionBin()
{

	//read myrobot directory
	char* myrobot_dir;
	myrobot_dir = getenv( "MYROBOT_DIR" );
	
	//read abs file name
	char* protect_file="protection.bin";
	
	//combine the above to one
	const int buf_size=50;
	char full_path[buf_size];

	sprintf(full_path,"%ssetting\\%s",myrobot_dir,protect_file);

	//remove protection.bin
	if( remove(full_path)!=0 )
		perror("error deleting protection.bin");
	else
		puts(" protection.bin successfully deleted ");

}

void MotionCard_ExamineProtect()
{

	//read myrobot directory
	char* myrobot_dir;
	myrobot_dir = getenv( "MYROBOT_DIR" );
	
	//read abs file name
	char* protect_file="protection.bin";
	
	//combine the above to one
	const int buf_size=50;
	char full_path[buf_size];

	sprintf(full_path,"%ssetting\\%s",myrobot_dir,protect_file);

	//print file path
	for( int i=0;i< buf_size; i++)
		printf( "%c",full_path[i] );
	
	int msgboxID;

	do
	{
		//check the file
		FILE* pfile;
		pfile=fopen( full_path, "r" );
		
		//no protection program
		if( pfile == NULL )
		{
			//if there is no protection.bin
			msgboxID = MessageBox( NULL, 
								  (LPCWSTR)"No Protection Program",
								  (LPCWSTR)"open protection before RETRY",
								  MB_ICONERROR | MB_RETRYCANCEL | MB_DEFBUTTON2 );

			if( msgboxID == IDCANCEL )
			{
 				exit( EXIT_FAILURE );
			}
		}
		else
		{
			//protection program opening
			msgboxID=-1;
			fclose( pfile );
		}

	}while( msgboxID == IDRETRY );

	printf("Protection program is open...\n");

}