#include "MyRobot_DrvComunicate.h" // �ݦb�̫e!
#include "MotionCard.h"


//=================== �����Ψ禡 ===================

void MotionCard_ABS( int Comport ) ; // ���﫬�s�X��
void MotionCard_GetEnc( int axis , long (&EncPulse) ) ; // Ū�s�X��pulse
void MotionCard_ClearProtectionBin() ;  // �O�@�{��
void MotionCard_ExamineProtect() ; // �O�@�{��


//==================== �}�Ҷb�d ====================

void MotionCard_OpenCard( int Comport )
{

	//MotionCard_ExamineProtect() ; // �O�@�{��

	int nRet = IMC_OpenDevice( 0 , 0 ) ; // �b�d��l��

	if( nRet == 0 )
	{

		printf( "Initialization Fails !" ) ;

	}
	else
	{

		IMC_GLB_ResetModule( RESET_ALL , 0 ) ; // ���m IMP �Ҳ�
		IMC_GLB_SetInterruptMask( IMC_ALL_INT_UNMASK , 0 ) ; // UnMask Interrupt

		//----------------- �s�X�� -----------------

		for( int i = 0 ; i < AXIS ; i++ )
		{

			IMC_ENC_SetInputFormat( i , ENC_FMT_AB , 0 ) ; // �]�w�s�X����J�榡�� A/B ��
			IMC_ENC_SetInputRate( i , ENC_RATE_X4 , 0 ) ; // �]�w�s�X����J�ѽX���v x4
			IMC_ENC_ClearCounter( i , 1 , 0 ) ; // �}�ҲM���s�X��
			IMC_ENC_ClearCounter( i , 0 , 0 ) ; // �����M���s�X��
			IMC_ENC_StartCounter( i , 1 , 0 ) ; // �}�l�s�X����J 

		}

		//MotionCard_ABS( Comport ) ; // Ū�����﫬�s�X��(Pulse)

		//---------------- DAC & ADC ----------------

		for( int i = 0 ; i < AXIS + 2  ; i++ )
		{

			IMC_DAC_SelectSource( i , DAC_SOURCE_SOFT , 0 ) ; // �]�w DAC �R�O�����n��W��
			IMC_DAC_EnableChannel( i , 1 , 0 ) ; // �}�� DAC �\��
			IMC_ADC_EnableChannel( i , 1 , 0 ) ; // �}�� ADC �\��	

		}

		IMC_ADC_SetConverterMode( 2 , 0 ) ;

		IMC_DAC_StartConverter( 1 , 0 ) ; // �}�l DAC �ഫ
		IMC_ADC_StartConverter( 1 , 0 ) ; // �}�l ADC �ഫ

		for( int i = 0 ; i < AXIS + 2  ; i++ )
		{

			IMC_DAC_SetOutputVoltage( i , 0.0 , 0 ) ; // DAC �q���M��

		}

		//------------------ RIO ------------------

		IMC_RIO_SetClockDivider( RIO_SET0 , 255 , 0 ) ; //�]�wClockDivider, IMP�w�]ClockDivider��255 ,���]�w�N�v�T�ǿ�t��, �B�����P�Ҧ��l���]�w�ۦP
		IMC_RIO_EnableSlaveControl( RIO_SET0 , RIO_SLAVE0 , 1 , 0 ) ; // �}��Slave�ǿ�\��
		IMC_RIO_EnableSetControl( RIO_SLAVE0 , 1 , 0 ) ;

		//---------------- ���_�\�� ----------------

		IMC_TMR_SetTimer( 1000 , 0 ) ; // �]�w�p�ɾ��p�ɮɶ� 1000us (1����)
		IMC_TMR_SetTimerEnable( 1 , 0 ) ; // �}�ҭp�ɾ��p�ɥ\��
		IMC_TMR_SetTimerIntEnable( 1, 0 ) ; // �}�ҭp�ɾ����_�\��

	}

}


//==================== �����b�d ====================

void MotionCard_CloseCard()
{

	IMC_TMR_SetTimerIntEnable( 0 , 0 ) ; // �����p�ɾ����_�\��
	IMC_TMR_SetTimerEnable( 0 , 0 ) ; // �����p�ɾ��p�ɥ\��

	for( int i = 0 ; i < AXIS  ; i++ )
	{

		IMC_DAC_SetOutputVoltage( i , 0.0 , 0 ) ; // DAC�q���M��

	}

	IMC_DAC_StartConverter( 0 , 0 ) ; // ���� DAC �ഫ
	IMC_ADC_StartConverter( 0 , 0 ) ; // ���� ADC �ഫ

	for( int i = 0 ; i < AXIS  ; i++ )
	{

		IMC_ENC_StartCounter( i , 0 , 0 ) ; // ����s�X����J
		IMC_DAC_EnableChannel( i , 0 , 0 ) ; // DAC���_�\��
		IMC_ADC_EnableChannel( i , 0 , 0 ) ; // ADC���_�\��

	}

	IMC_RIO_EnableSlaveControl( RIO_SET0,RIO_SLAVE0 , 0 , 0 ) ; // ����Slave�ǿ�\��
	IMC_RIO_EnableSetControl( RIO_SLAVE0 , 0 , 0 ) ;

	IMC_CloseIfOpen( 0 ) ; // ���� IMP �Ҳ�

	WinExec( "taskkill /im safety_monitoring.exe", SW_HIDE ) ; // kill protection program �O�@�{��
	MotionCard_ClearProtectionBin() ; // �O�@�{��

}


//============= Servo On (Torque Mode) =============

void MotionCard_ServoOn()
{
	
	DWORD input ;
	do
	{
		IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT2 , 0x0f09 , 0 ) ; // �g�J Servo ON
		IMC_RIO_GetInputValue( RIO_SET0 , RIO_SLAVE0 , RIO_PORT2 , &input , 0 ) ; // Ū�� �T�{�O�_ Servo ON

	}while( input && ( 0x0100 ) != 0x0100 ) ;
	printf( "servo on success\n" ) ;
	
	IMC_RIO_SetOutputValue( RIO_SET0 , RIO_SLAVE0 , RIO_PORT0 , 0x3500 , 0 ) ; // �b 1,3,5,6 ���F�ϦV��
	IMC_RIO_SetOutputValue( RIO_SET0 , RIO_SLAVE0 , RIO_PORT1 , 0x000a , 0 ) ; // �b 2,4 ���F���V��

}


//==================== Servo Off ====================

void MotionCard_ServoOff()
{

	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT2 , 0x0000 , 0) ; // RIO_PORT2( Servo off�� ) ���u�������B�ݥ[ Sleep
	Sleep( 50 ) ;
	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT1 , 0x0000 , 0) ; // RIO_PORT1( ABSR + BRK )
	Sleep( 50 ) ;
	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT0 , 0x0000 , 0) ; // RIO_PORT0( Reset + ABSM )
	Sleep( 50 ) ;
	IMC_RIO_SetOutputValue(RIO_SET0 , RIO_SLAVE0 , RIO_PORT3 , 0x0000 , 0) ; // RIO_PORT3( ������ + ��L )
	Sleep( 50 ) ;

}


//================= Ū�����c���׭� =================

double ABSPulse[AXIS] ;

void MotionCard_ABS( int Comport ) // ���F���﫬�s�X��(Ū����T�w��)
{

	static double DirectionABS[AXIS] = { -1 , 1 , -1 , 1 , -1 , -1 } ;
	MyRobot_DrvComunicate* drv_talk = new MyRobot_DrvComunicate( Comport , 9600 ) ;

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		int ReadPulse = 0 ;
		drv_talk->MyRobot_ABS_Position( i , ReadPulse ) ;
		ABSPulse[i] = DirectionABS[i] * ReadPulse ;  // ���﫬�s�X����(Pulse)

	}
	
	delete drv_talk ;

}

void MotionCard_GetEnc( int axis , long (&MotorEncPulse) ) // ���F�W�q+����s�X����
{

	long lCount ;

	IMC_ENC_ReadCounter( axis, &lCount, 0 ) ; // Ū���W�q��
	MotorEncPulse = lCount + (long)ABSPulse[axis] ; // �W�q+����s�X����(Pulse)

}

void MotionCard_Encoder( double (& RobotPosRad)[AXIS] ) // ���c���׭�(rad)
{

	const double CircleRad = 2 * PI ;
	long MotorEncPulse[AXIS] = { 0 } ;

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		MotionCard_GetEnc( i , MotorEncPulse[i] ) ; // Ū���W�q+�����
		RobotPosRad[i] = ( (double)MotorEncPulse[i] / 3600000.0 ) * CircleRad ; // ���c����(Rad)�A���c�@��3600000��Pulse(�H�X�ʾ��Ѽƽվ�)

	}

}


//====================== DAC ======================

const double GearRatio[AXIS] = { 120.0 , 120.0 , 120.0 , 102.0 , 80.0 , 51.0 } ; // ��t��
const double RatedTorque[AXIS] = { 1.3 , 1.3 , 1.3 , 0.32 , 0.32 , 0.32 } ; // ���F�B�w��x
//const double MaximumTorque[AXIS] = { 3.8 , 3.8 , 3.8 , 0.95 , 0.95 , 0.95 } ; // ���F�̤j��x
const double MaxVoltage = 8 ; // �̤j�q��

void MotionCard_DAC( double RobotTorCtrl[AXIS] )
{

	for ( int i = 0 ; i < AXIS ; i++ )
	{

		double MotorTorCtrl = RobotTorCtrl[i] / GearRatio[i] ; // ���F��x����R�O(Nm)

		if ( MotorTorCtrl > RatedTorque[i] )
		{
			MotorTorCtrl = RatedTorque[i] ; // ���F��x����
		}

		if ( MotorTorCtrl < -RatedTorque[i] )
		{
			MotorTorCtrl = -RatedTorque[i] ; // ���F��x����
		}

		double ControlSignal = ( MotorTorCtrl / RatedTorque[i] ) * MaxVoltage ; // ����T��(V): �̤j�q������(�B�w�γ̤j)��x(��TLC�]�w)
		
		IMC_DAC_SetOutputVoltage( i , (float)ControlSignal , 0 ) ;

	}

}


//====================== ADC ======================

void MotionCard_ADC( float (&ADCVoltage)[8] )
{

	//float ADCVoltage[AXIS] ;

	for ( int i = 0 ; i < 8 ; i++ )
	{

		IMC_ADC_GetInputVoltage( i , &ADCVoltage[i] , 0 ) ; // Ū��ADC�q��
		//ADCVoltage[i] = ADCVoltage[i] + ( 0.03482 * ADCVoltage[i] - 0.016577 ) ; // ADC���m���v (Offset)

	}

}


//=================== �������_ ===================

void MotionCard_ChangeTimer( TMRISR Timer )
{

	IMC_TMR_SetISRFunction( Timer , 0 ) ; //�걵���_�A�Ȩ禡

}


//=================== �O�@�{�� ===================

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