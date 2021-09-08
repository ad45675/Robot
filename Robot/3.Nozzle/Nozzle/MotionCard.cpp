#include "MotionCard.h"




void MotionCard_OpenCard()  // �}�d�ʧ@
{

	int nRet = IMC_OpenDevice( 0 , 0 ) ; // ��l��IMP-2�B�ʱ���d

	if( nRet == 0 )
	{
		printf( "Initialization Fails" ) ;
	}
	else
	{
		IMC_GLB_ResetModule( RESET_ALL , 0 ) ; //  ���m IMP �Ҳ�
		IMC_GLB_SetInterruptMask( IMC_ALL_INT_UNMASK , 0 ) ; //UnMask Interrupt


		for( int i = 0 ; i < 8 ; i++ )
		{
			IMC_ENC_SetInputFormat( i , ENC_FMT_AB , 0 ) ;	  //  �]�w�s�X����J�榡�� A/B ��
			IMC_ENC_SetInputRate( i , ENC_RATE_X4 , 0 ) ;     //  �]�w�s�X����J�ѽX���v x4
			IMC_ENC_ClearCounter( i , 1 , 0 ) ;               //  �}�ҲM���s�X��
			IMC_ENC_ClearCounter( i , 0 , 0 ) ;               //  �����M���s�X��
			IMC_ENC_StartCounter( i , 1 , 0 ) ;               //  �}�l�s�X����J 
		}

		for( int i = 0 ; i < 8 ; i++ )
		{
			IMC_DAC_SelectSource( i , DAC_SOURCE_SOFT, 0 ) ;  // �]�w DAC �R�O�����n��W��
			IMC_DAC_EnableChannel( i , 1 , 0 ) ;              // �}�� DAC �\��
		}
		IMC_DAC_StartConverter( 1 , 0 ) ;   // �}�l DAC �ഫ


		IMC_ADC_SetConverterMode( 2 , 0 ) ;
		for( int i = 0 ; i < 8 ; i++ )
		{
			IMC_ADC_EnableChannel( i , 1 , 0 ) ;  // �}�� ADC �\��
		}
		IMC_ADC_StartConverter( 1 , 0 ) ;	// �}�l ADC �ഫ


		IMC_RIO_SetClockDivider( RIO_SET0 , 255 , 0 ) ;  //�]�wClockDivider, IMP�w�]ClockDivider��255 ,���]�w�N�v�T�ǿ�t��, �B�����P�Ҧ��l���]�w�ۦP
		IMC_RIO_EnableSlaveControl( RIO_SET0 , RIO_SLAVE0 , 1 , 0 ) ;	//  �}��Slave�ǿ�\��
		IMC_RIO_EnableSetControl( RIO_SLAVE0 , 1 , 0 ) ;

		IMC_TMR_SetTimer( 1000 , 0 ) ;		 //  �]�w�p�ɾ��p�ɮɶ� 1000 us (1����)
		IMC_TMR_SetTimerEnable( 1 , 0 ) ;	 //  �}�ҭp�ɾ��p�ɥ\��
		IMC_TMR_SetTimerIntEnable( 1, 0 ) ;	 //  �}�ҭp�ɾ����_�\��
		
	}

}


void MotionCard_CloseCard( )  // ���d�ʧ@
{
	IMC_TMR_SetTimerIntEnable( 0 , 0 ) ;  //  �����p�ɾ����_�\��
	IMC_TMR_SetTimerEnable( 0 , 0 ) ;	  //  �����p�ɾ��p�ɥ\��

	for( int i = 0 ; i < 8 ; i++ )
	{
		IMC_ENC_StartCounter( i , 0 , 0 ) ;        // ����s�X����J
		IMC_DAC_SetOutputVoltage( i , 0.0 , 0 ) ;  // DAC�q���M��
		IMC_ADC_EnableChannel( i , 0 , 0 ) ;       // ADC���_�\��
	}

	IMC_DAC_StartConverter( 0 , 0 ) ;  // ���� DAC �ഫ
	IMC_ADC_StartConverter( 0 , 0 ) ;  // ���� ADC �ഫ

	IMC_RIO_EnableSlaveControl( RIO_SET0,RIO_SLAVE0 , 0 , 0 ) ;	 //  ����Slave�ǿ�\��
	IMC_RIO_EnableSetControl( RIO_SLAVE0 , 0 , 0 ) ;

	IMC_CloseIfOpen( 0 ) ;	//  ���� IMP �Ҳ�

}






void MotionCard_ChangeTimer( TMRISR Timer ) // �������P���_�{��
{
	IMC_TMR_SetISRFunction( Timer , 0 ) ;  //�걵���_�A�Ȩ禡
}

//====================== ADC ======================

void MotionCard_ADC(float(&ADCVoltage)[8])
{

	for (int i = 0; i < 8; i++)
	{

		IMC_ADC_GetInputVoltage(i, &ADCVoltage[i], 0); // Ū��ADC�q��

	}

}
