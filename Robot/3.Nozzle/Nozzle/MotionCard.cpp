#include "MotionCard.h"




void MotionCard_OpenCard()  // 開卡動作
{

	int nRet = IMC_OpenDevice( 0 , 0 ) ; // 初始化IMP-2運動控制卡

	if( nRet == 0 )
	{
		printf( "Initialization Fails" ) ;
	}
	else
	{
		IMC_GLB_ResetModule( RESET_ALL , 0 ) ; //  重置 IMP 模組
		IMC_GLB_SetInterruptMask( IMC_ALL_INT_UNMASK , 0 ) ; //UnMask Interrupt


		for( int i = 0 ; i < 8 ; i++ )
		{
			IMC_ENC_SetInputFormat( i , ENC_FMT_AB , 0 ) ;	  //  設定編碼器輸入格式為 A/B 型
			IMC_ENC_SetInputRate( i , ENC_RATE_X4 , 0 ) ;     //  設定編碼器輸入解碼倍率 x4
			IMC_ENC_ClearCounter( i , 1 , 0 ) ;               //  開啟清除編碼器
			IMC_ENC_ClearCounter( i , 0 , 0 ) ;               //  關閉清除編碼器
			IMC_ENC_StartCounter( i , 1 , 0 ) ;               //  開始編碼器輸入 
		}

		for( int i = 0 ; i < 8 ; i++ )
		{
			IMC_DAC_SelectSource( i , DAC_SOURCE_SOFT, 0 ) ;  // 設定 DAC 命令源為軟體規劃
			IMC_DAC_EnableChannel( i , 1 , 0 ) ;              // 開啟 DAC 功能
		}
		IMC_DAC_StartConverter( 1 , 0 ) ;   // 開始 DAC 轉換


		IMC_ADC_SetConverterMode( 2 , 0 ) ;
		for( int i = 0 ; i < 8 ; i++ )
		{
			IMC_ADC_EnableChannel( i , 1 , 0 ) ;  // 開啟 ADC 功能
		}
		IMC_ADC_StartConverter( 1 , 0 ) ;	// 開始 ADC 轉換


		IMC_RIO_SetClockDivider( RIO_SET0 , 255 , 0 ) ;  //設定ClockDivider, IMP預設ClockDivider為255 ,此設定將影響傳輸速度, 且必須與所有子版設定相同
		IMC_RIO_EnableSlaveControl( RIO_SET0 , RIO_SLAVE0 , 1 , 0 ) ;	//  開啟Slave傳輸功能
		IMC_RIO_EnableSetControl( RIO_SLAVE0 , 1 , 0 ) ;

		IMC_TMR_SetTimer( 1000 , 0 ) ;		 //  設定計時器計時時間 1000 us (1豪秒)
		IMC_TMR_SetTimerEnable( 1 , 0 ) ;	 //  開啟計時器計時功能
		IMC_TMR_SetTimerIntEnable( 1, 0 ) ;	 //  開啟計時器中斷功能
		
	}

}


void MotionCard_CloseCard( )  // 關卡動作
{
	IMC_TMR_SetTimerIntEnable( 0 , 0 ) ;  //  關閉計時器中斷功能
	IMC_TMR_SetTimerEnable( 0 , 0 ) ;	  //  關閉計時器計時功能

	for( int i = 0 ; i < 8 ; i++ )
	{
		IMC_ENC_StartCounter( i , 0 , 0 ) ;        // 停止編碼器輸入
		IMC_DAC_SetOutputVoltage( i , 0.0 , 0 ) ;  // DAC電壓清空
		IMC_ADC_EnableChannel( i , 0 , 0 ) ;       // ADC中斷功能
	}

	IMC_DAC_StartConverter( 0 , 0 ) ;  // 停止 DAC 轉換
	IMC_ADC_StartConverter( 0 , 0 ) ;  // 停止 ADC 轉換

	IMC_RIO_EnableSlaveControl( RIO_SET0,RIO_SLAVE0 , 0 , 0 ) ;	 //  關閉Slave傳輸功能
	IMC_RIO_EnableSetControl( RIO_SLAVE0 , 0 , 0 ) ;

	IMC_CloseIfOpen( 0 ) ;	//  關閉 IMP 模組

}






void MotionCard_ChangeTimer( TMRISR Timer ) // 切換不同中斷程式
{
	IMC_TMR_SetISRFunction( Timer , 0 ) ;  //串接中斷服務函式
}

//====================== ADC ======================

void MotionCard_ADC(float(&ADCVoltage)[8])
{

	for (int i = 0; i < 8; i++)
	{

		IMC_ADC_GetInputVoltage(i, &ADCVoltage[i], 0); // 讀取ADC電壓

	}

}
