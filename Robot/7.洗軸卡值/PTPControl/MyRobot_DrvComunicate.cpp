#include "stdafx.h"
#include "MyRobot_DrvComunicateDef.h"
#include "MyRobot_DrvComunicate.h"
#include<math.h>

MyRobot_DrvComunicate::MyRobot_DrvComunicate(int com_port, int baud_rate)
{
	char str[5];
	str[0]='C';
	str[1]='O';
	str[2]='M';

	if(com_port==0)
	{
		str[3]='0';
	}
	else if(com_port==1)
	{
		str[3]='1';
	}
	else if(com_port==2)
	{
		str[3]='2';
	}
	else if(com_port==3)
	{
		str[3]='3';
	}
	else if(com_port==4)
	{
		str[3]='4';
	}
	else if(com_port==5)
	{
		str[3]='5';
	}
	else if(com_port==6)
	{
		str[3]='6';
	}
	else if(com_port==7)
	{
		str[3]='7';
	}
	else if(com_port==8)
	{
		str[3]='8';
	}
	else if(com_port==9)
	{
		str[3]='9';
	}
	str[4]='\0';
	myRobotProtocol=new MyRobot_Protocol(str,baud_rate);
}

MyRobot_DrvComunicate::~MyRobot_DrvComunicate()
{
	delete myRobotProtocol;
}

void MyRobot_DrvComunicate:: getCheckSum(std::vector<uchar> *instruction)
{
	int n=(*instruction).size();
	int total=0;
	for(int i=1;i<=n-1;i++)
	{
		total+=(*instruction)[i];
	}
	int sum=total;
	int checkSum[2];
	checkSum[1]=sum%16;
	sum=sum/16;
	checkSum[0]=sum%16;
	uchar checkSumForChar[2];
	for(int i=0;i<=1;i++)
	{
		switch(checkSum[i])
		{
		case 0:
			checkSumForChar[i]='0';
			break;
		case 1:
			checkSumForChar[i]='1';
			break;
		case 2:
			checkSumForChar[i]='2';
			break;
		case 3:
			checkSumForChar[i]='3';
			break;
		case 4:
			checkSumForChar[i]='4';
			break;
		case 5:
			checkSumForChar[i]='5';
			break;
		case 6:
			checkSumForChar[i]='6';
			break;
		case 7:
			checkSumForChar[i]='7';
			break;
		case 8:
			checkSumForChar[i]='8';
			break;
		case 9:
			checkSumForChar[i]='9';
			break;
		case 10:
			checkSumForChar[i]='A';
			break;
		case 11:
			checkSumForChar[i]='B';
			break;
		case 12:
			checkSumForChar[i]='C';
			break;
		case 13:
			checkSumForChar[i]='D';
			break;
		case 14:
			checkSumForChar[i]='E';
			break;
		case 15:
			checkSumForChar[i]='F';
			break;
		}
	}

	(*instruction).push_back(checkSumForChar[0]);
	(*instruction).push_back(checkSumForChar[1]);
}

std::vector<uchar> MyRobot_DrvComunicate:: produceInstruction(uchar stationNumber,uchar cmd1,uchar cmd2,uchar dataNO1,uchar dataNO2)
{
	std::vector<uchar> instruction;
	instruction.push_back(SOH);
	instruction.push_back(stationNumber);
	instruction.push_back(cmd1);
	instruction.push_back(cmd2);
	instruction.push_back(STX);
	instruction.push_back(dataNO1);
	instruction.push_back(dataNO2);
	instruction.push_back(ETX);
	//getCheckSum(&instruction);
	return instruction;
}


std::vector<uchar> MyRobot_DrvComunicate:: produceInstruction(uchar stationNumber,uchar cmd1,uchar cmd2,uchar dataNO1,uchar dataNO2,uchar *data)
{
	if(data==NULL)
	{
		
		return produceInstruction(stationNumber,cmd1,cmd2,dataNO1,dataNO2);
	}
	std::vector<uchar> instruction;
	instruction.push_back(SOH);
	instruction.push_back(stationNumber);
	instruction.push_back(cmd1);
	instruction.push_back(cmd2);
	instruction.push_back(STX);
	instruction.push_back(dataNO1);
	instruction.push_back(dataNO2);
	int i=0;
	while(data[i]!='\0')
	{
		instruction.push_back(data[i]);
		i++;
	}
	instruction.push_back(ETX);
	//getCheckSum(&instruction);
	return instruction;
}

void MyRobot_DrvComunicate:: readCommand(std::vector<uchar> reply)
{
	std::cout<<"以下為readCommand的內容---------------------------------------------------"<<std::endl;
	switch(reply[2])
	{
	case 'a':
		std::cout<<"servo alarm but normal operation"<<std::endl;

		if(reply.size()>6)
		{
			int n=reply.size()-6;
			uchar *str=new uchar[n+1];
			for(int i=0;i<=n-1;i++)
			{
				str[i]=reply[i+3];
			}
			str[n]='\0';
			std::cout<<"station number="<<reply[1]<<" 傳來的字串為"<<str<<std::endl;
		}
		else
		{
			std::cout<<"station number="<<reply[1]<<" 無傳來的字串"<<std::endl;
		}

		break;
	case 'b':
		std::cout<<"servo alarm and parity error"<<std::endl;
		break;
	case 'c':
		std::cout<<"servo alarm and checksum error"<<std::endl;
		break;
	case 'd':
		std::cout<<"servo alarm and character error"<<std::endl;
		break;
	case 'e':
		std::cout<<"servo alarm and command error"<<std::endl;
		break;
	case 'f':
		std::cout<<"servo alarm and data N.O error"<<std::endl;
		break;
	case 'B':
		std::cout<<"servo normal and parity error"<<std::endl;
		break;
	case 'C':
		std::cout<<"servo normal and checksum error"<<std::endl;
		break;
	case 'D':
		std::cout<<"servo normal and character error"<<std::endl;
		break;
	case 'E':
		std::cout<<"servo normal and command error"<<std::endl;
		break;
	case 'F':
		std::cout<<"servo normal and data N.O error"<<std::endl;
		break;
	case 'A':
		if(reply.size()>6)
		{
			int n=reply.size()-6;
			uchar *str=new uchar[n+1];
			for(int i=0;i<=n-1;i++)
			{
				str[i]=reply[i+3];
			}
			str[n]='\0';
			std::cout<<"station number="<<reply[1]<<" 傳來的字串為"<<str<<std::endl;
		}
		else
		{
			std::cout<<"station number="<<reply[1]<<" 無傳來的字串"<<std::endl;
		}
		break;
	}
	
	

	std::cout<<"以上為readCommand的內容******************************************************"<<std::endl;
}

uchar MyRobot_DrvComunicate::int2char(int n)
{
	char temp[20];
	itoa(n,temp,10);
	return temp[0];
}

int  MyRobot_DrvComunicate:: char2int(uchar c)
{
	if(c=='A'||c=='a')
	{
		return 10;
	}
	else if(c=='B'||c=='b')
	{
		return 11;
	}
	else if(c=='C'||c=='c')
	{
		return 12;
	}
	else if(c=='D'||c=='d')
	{
		return 13;
	}
	else if(c=='E'||c=='e')
	{
		return 14;
	}
	else if(c=='F'||c=='f')
	{
		return 15;
	}
	else if(c=='0')
	{
		return 0;
	}
	else if(c=='1')
	{
		return 1;
	}
	else if(c=='2')
	{
		return 2;
	}
	else if(c=='3')
	{
		return 3;
	}
	else if(c=='4')
	{
		return 4;
	}
	else if(c=='5')
	{
		return 5;
	}
	else if(c=='6')
	{
		return 6;
	}
	else if(c=='7')
	{
		return 7;
	}
	else if(c=='8')
	{
		return 8;
	}
	else if(c=='9')
	{
		return 9;
	}
}

int calculateStrLength(char *str)
{
	int i;
	for(i=0;str[i]!='\0';i++);
	return i;

}

double MyRobot_DrvComunicate:: hex2dec(char str[])
{
	
	int strLength=strlen(str);
	double total=0;
	for(int i=0;i<=strLength-1;i++)
	{
		total+=char2int(str[i])*powf(16,strLength-1-i);
	}
	if(str[0]=='F'||str[0]=='f')
	{
		return -(powf(16,strLength)-total);
	}
	else
	{
		return total;
	}
}


double MyRobot_DrvComunicate::returnStatusDisplayDataValue(std::vector<uchar> reply)
{
	double num;
	if(reply[6]=='0')
	{
		char str[]={reply[7],reply[8],reply[9],reply[10],reply[11],reply[12],reply[13],reply[14],'\0'};
		num=hex2dec(str);
		if(reply[5]!='0')
		{
			num=num*powf(10,-1*(char2int(reply[5])-1));
		}
	}
	else if(reply[6]=='1')
	{
		char str[]={reply[7],reply[8],reply[9],reply[10],reply[11],reply[12],reply[13],reply[14],'\0'};
		num=0;
		for(int i=0;i<=7;i++)
		{
			num=num+char2int(str[i])*powf(10,7-i);
		}
		if(reply[5]!='0')
		{
			num=num*powf(10,-1*(char2int(reply[5])-1));
		}
	}
	return num;
}

double MyRobot_DrvComunicate::returnABSPositionDataValue(std::vector<uchar> reply)
{
	double num=0;
	
		char str[]={reply[3],reply[4],reply[5],reply[6],reply[7],reply[8],reply[9],reply[10],'\0'};
		num=hex2dec(str);
		
	return num;
}

void MyRobot_DrvComunicate::MyRobot_ReadABS_Counter(int station_num,int &pulse)
{
	
	std::vector<uchar> instruction=produceInstruction(int2char(station_num),'0','1','8','C');
	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
	readCommand(instruction);
	if(status==true)
	{
		pulse=(int)(returnStatusDisplayDataValue(instruction));
	}
	else
	{
		std::cout<<"沒傳成功"<<std::endl;
	}
}

void MyRobot_DrvComunicate::MyRobot_CumulativeCmdPulse(int station_num,int &pulse)
{
	
	std::vector<uchar> instruction=produceInstruction(int2char(station_num),'0','1','8','3');
	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
	readCommand(instruction);
	if(status==true)
	{
		pulse=(int)(returnStatusDisplayDataValue(instruction));
	}
	else
	{
		std::cout<<"沒傳成功"<<std::endl;
	}
}

void MyRobot_DrvComunicate::MyRobot_Within1RevPosition(int station_num,int &pulse)
{
	
	std::vector<uchar> instruction=produceInstruction(int2char(station_num),'0','1','8','B');
	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
	readCommand(instruction);
	if(status==true)
	{
		pulse=(int)(returnStatusDisplayDataValue(instruction));
	}
	else
	{
		std::cout<<"沒傳成功"<<std::endl;
	}
}

void MyRobot_DrvComunicate::MyRobot_DroopPulse(int station_num,int &pulse)
{
	
	std::vector<uchar> instruction=produceInstruction(int2char(station_num),'0','1','8','2');
	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
	readCommand(instruction);
	if(status==true)
	{
		pulse=(int)(returnStatusDisplayDataValue(instruction));
	}
	else
	{
		std::cout<<"沒傳成功"<<std::endl;
	}
}

void MyRobot_DrvComunicate::MyRobot_ABS_Position(int station_num,int &pulse)
{
	
	std::vector<uchar> instruction=produceInstruction(int2char(station_num),'0','2','9','1');
	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
	readCommand(instruction);
	if(status==true)
	{
		pulse=returnABSPositionDataValue(instruction);
	}
	else
	{
		std::cout<<"沒傳成功"<<std::endl;
	}
}

//void MyRobot_DrvComunicate::MyRobot_Communicate(char station_num, char cmd_num1,char cmd_num2, char data_num1,char data_num2, uchar *data )
//{
//	std::vector<uchar> instruction=produceInstruction(station_num,cmd_num1,cmd_num2,data_num1,data_num2,data);
//	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
//	
//	printf("以下為readCommand\n");
//	if(status==true)
//	{
//		
//		readCommand(instruction);
//		
//	}
//	else
//	{
//		std::cout<<"沒傳成功"<<std::endl;
//	}
//	printf("------------------------------");
//}


bool MyRobot_DrvComunicate:: MyRobot_ABS2Incre( int station_Num )
{
	bool bSetBasicSettingParameter=MyRobot_Set2BasicSettingParameter(station_Num);

	if(bSetBasicSettingParameter==true)
	{
		uchar valueOfABS2Inc[]={'0','0','0','0','0','0','0','0','\0'};
		/*std::vector<uchar> setABS2INCins=produceInstruction(int2char(station_Num),'8','4','0','3',valueOfABS2Inc);
		status=myRobotProtocol->Protocol_communicate(setABS2INCins,2);*/
		bool status=MyRobot_Communicate(station_Num,'8','4','0','3',valueOfABS2Inc);
		return status;
	}
	else
	{
		return false;
	}
}

bool MyRobot_DrvComunicate::MyRobot_Incre2ABS( int station_Num )
{
	bool bSetBasicSettingParameter=MyRobot_Set2BasicSettingParameter(station_Num);
	
	if(bSetBasicSettingParameter==true)
	{
		uchar valueOfINC2ABS[]={'0','0','0','0','0','0','0','1','\0'};
		/*std::vector<uchar> setINC2ABSins=produceInstruction(int2char(station_Num),'8','4','0','3',valueOfINC2ABS);
		status=myRobotProtocol->Protocol_communicate(setINC2ABSins,2);*/
		bool status=MyRobot_Communicate(station_Num,'8','4','0','3',valueOfINC2ABS);
		return status;
	}
	else
	{
		return false;
	}		
}


bool MyRobot_DrvComunicate::MyRobot_toVmode( int station_Num )
{
	bool bSetBasicSettingParameter=MyRobot_Set2BasicSettingParameter(station_Num);
	
	if(bSetBasicSettingParameter==true)
	{
		uchar valueP2V[]={'0','0','0','0','0','0','0','2','\0'};
		/*std::vector<uchar> setP2Vins=produceInstruction(int2char(station_Num),'8','4','0','1',valueP2V);
		status=myRobotProtocol->Protocol_communicate(setP2Vins,2);*/
		bool status=MyRobot_Communicate(station_Num,'8','4','0','1',valueP2V);
		
		return status;
	}
	else
	{
		return false;
	}
}

bool MyRobot_DrvComunicate::MyRobot_toPmode( int station_Num )
{
	bool bSetBasicSettingParameter=MyRobot_Set2BasicSettingParameter(station_Num);
	
	if(bSetBasicSettingParameter==true)
	{
		uchar valueV2P[]={'0','0','0','0','0','0','0','0','\0'};
		/*std::vector<uchar> setV2Pins=produceInstruction(int2char(station_Num),'8','4','0','1',valueV2P);
		status=myRobotProtocol->Protocol_communicate(setV2Pins,2);*/
		bool status=MyRobot_Communicate(station_Num,'8','4','0','1',valueV2P);		
		return status;
	}
	else
	{
		return false;
	}
	
}

bool MyRobot_DrvComunicate::MyRobot_toTorqmode(int station_Num)
{
	bool bSetBasicSettingParameter=MyRobot_Set2BasicSettingParameter(station_Num);
	
	if(bSetBasicSettingParameter==true)
	{
		uchar value2T[]={'0','0','0','0','0','0','0','4','\0'};
		/*std::vector<uchar> setV2Pins=produceInstruction(int2char(station_Num),'8','4','0','1',valueV2P);
		status=myRobotProtocol->Protocol_communicate(setV2Pins,2);*/
		bool status=MyRobot_Communicate(station_Num,'8','4','0','1',value2T);		
		return status;
	}
	else
	{
		return false;
	}
}

bool MyRobot_DrvComunicate::MyRobot_Communicate( int station_num, char cmd_num1,char cmd_num2, char data_num1,char data_num2, uchar *data )
{
	std::vector<uchar> instruction=produceInstruction(int2char( station_num),cmd_num1,cmd_num2,data_num1,data_num2,data);
	bool status=myRobotProtocol->Protocol_communicate(instruction,2);
	printf("以下是送指令給驅動器，驅動器回傳的東西\n");
	if(status==true)
	{			
		readCommand(instruction);			
			
	}
	else
	{
		std::cout<<cmd_num1<<" "<<cmd_num2<<" "<<data_num1<<" "<<data_num2<<" "<<data<<" 指令失敗"<<std::endl;
	}
	printf("以上是驅動器回傳的東西-------------------------------------");
	return status;
}

bool MyRobot_DrvComunicate::MyRobot_Set2BasicSettingParameter(int station_Num)
{
	uchar valueOfParaWrite[]={'0','0','0','0','\0'};
	std::vector<uchar> setParaGroupIns=produceInstruction(int2char(station_Num),'8','5','0','0',valueOfParaWrite);
	bool status=myRobotProtocol->Protocol_communicate(setParaGroupIns,2);

	printf("以下是設定parameter group傳回的東西\n");
	if(status==true)
	{
		
		readCommand(setParaGroupIns);
		
	}
	else
	{
		printf("修改parameter group宣告失敗\n");

	}
	printf("----------------------------------------\n");
	return status;
}