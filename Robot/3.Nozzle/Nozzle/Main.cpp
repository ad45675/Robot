#include "MotionCard.h"
#include "Nozzle.h"

int Comport = 4; //USB comport
int TimerFlag = 0;
double StayCount = 0.0;
double StayCount2 = 0.0;
float ADCVoltage[8] = { 0.0 };
int ADC_port = 6;

using namespace std;

void _stdcall Nozzle_Suck(TMRINT *pst);
void _stdcall Nozzle_Release(TMRINT *pst);
void _stdcall Nothing(TMRINT *pst);
void _stdcall test(TMRINT *pst);

int main()
{

	//-------------------- �}�ҦU���\�� --------------------

	printf("Press any key to start !\n");
	printf("Press 'Esc' to end program.\n");

	while (_kbhit() == 0); // �����N��}�l�{��

	MotionCard_OpenCard(); // �}�Ҷb�d

	//---------------------- �u�@�y�{ ----------------------

	int sw = 0;
	int close = 0;

	while (sw != 0x1b)
	{                                           

		//-------------------- ��ʵ����{�� --------------------

		if (_kbhit())
		{

			sw = _getch(); // ��ESC�����{��
			close = sw; // �� �ťի� ��}

		}
		
		//---------------------- �������_ ----------------------
		
		if (TimerFlag == 0)
		{

			MotionCard_ChangeTimer(Nozzle_Suck);

		}

		else if (TimerFlag == 1)
		{
			MotionCard_ChangeTimer(Nothing);
			if (close == 0x20)
			{
				printf("��}\n");
				TimerFlag = 2;
			}
		}
		else if (TimerFlag == 2)
		{

			MotionCard_ChangeTimer(Nozzle_Release);

		}

		else if (TimerFlag == 3)
		{
			break;
		}

		else if (TimerFlag == 4)
		{
			MotionCard_ChangeTimer(test);
			if (close == 0x20)
			{
				printf("��}\n");
				TimerFlag = 2;
			}
		}

		


	}

	//-------------------- �����U���\�� --------------------

	MotionCard_CloseCard(); // �����b�d
	system("Pause");
}



void _stdcall Nozzle_Suck(TMRINT *pst)
{
	if (TimerFlag == 0 && StayCount <10000)
	{
		Nozzle_Suck();

		MotionCard_ADC(ADCVoltage);
	}
	else
	{

		TimerFlag = 2; // ��}

	}

	//cout <<"release"<< ADCVoltage[6] << "\t" <<"suck"<<  ADCVoltage[7] <<"\n"<<endl;
	 
	if (ADCVoltage[ADC_port] > 2)
	{
		printf("�l��F\n");
		TimerFlag = 1;
	}

	printf("StayCount %f \r", StayCount);
	
	StayCount++;
}

void _stdcall Nothing(TMRINT *pst)
{
	TimerFlag = 4;
}

void _stdcall test(TMRINT *pst)
{
	cout<<"suction"<<endl;
}

void _stdcall Nozzle_Release(TMRINT *pst)
{
	if (StayCount2 < 1000)
	{
		Nozzle_Release();

	}
	else
	{
		cout <<"release"<< ADCVoltage[6] << "\t" <<"suck"<<  ADCVoltage[7] <<"\n"<<endl;
		TimerFlag = 3;
	}

	StayCount2++;
}