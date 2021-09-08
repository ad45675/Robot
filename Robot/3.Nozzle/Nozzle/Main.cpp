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

	//-------------------- 開啟各項功能 --------------------

	printf("Press any key to start !\n");
	printf("Press 'Esc' to end program.\n");

	while (_kbhit() == 0); // 按任意鍵開始程式

	MotionCard_OpenCard(); // 開啟軸卡

	//---------------------- 工作流程 ----------------------

	int sw = 0;
	int close = 0;

	while (sw != 0x1b)
	{                                           

		//-------------------- 手動結束程式 --------------------

		if (_kbhit())
		{

			sw = _getch(); // 按ESC結束程式
			close = sw; // 按 空白建 放開

		}
		
		//---------------------- 切換中斷 ----------------------
		
		if (TimerFlag == 0)
		{

			MotionCard_ChangeTimer(Nozzle_Suck);

		}

		else if (TimerFlag == 1)
		{
			MotionCard_ChangeTimer(Nothing);
			if (close == 0x20)
			{
				printf("放開\n");
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
				printf("放開\n");
				TimerFlag = 2;
			}
		}

		


	}

	//-------------------- 關閉各項功能 --------------------

	MotionCard_CloseCard(); // 關閉軸卡
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

		TimerFlag = 2; // 放開

	}

	//cout <<"release"<< ADCVoltage[6] << "\t" <<"suck"<<  ADCVoltage[7] <<"\n"<<endl;
	 
	if (ADCVoltage[ADC_port] > 2)
	{
		printf("吸住了\n");
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