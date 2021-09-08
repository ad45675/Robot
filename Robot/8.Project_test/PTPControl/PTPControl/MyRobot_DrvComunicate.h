
/*
*	The Class contains functions for driver (mitsubishi) communication 
*	Author: Howard
*	Version: v1.2
*	Note: verified-(+), non-verified-(-)
*	History: 2014/04/25 release version
*			 2014/04/29 add the ifdef to protect this library
*
*/

#ifndef MYROBOT_DRVCOMUNICATE_H
#define	MYROBOT_DRVCOMUNICATE_H

#include  "MyRobot_Protocol.h"

class MyRobot_DrvComunicate
{

private:
	MyRobot_Protocol *myRobotProtocol;
public:
	/*
	* The com_port and baudrate are specified and the communication is ready
	*/
	MyRobot_DrvComunicate( int com_port, int baud_rate );
	
	/*
	* destruct the communication
	*/
	~MyRobot_DrvComunicate();
	
public:
	//read ABS Counter
	//>station number(0~5)
	//<pulse(the value of ABS Counter
	void MyRobot_ReadABS_Counter( int station_num, int& pulse );	
	//
	//change ABS encoder to Increment encoder
	//>station number(0~5)
	//<bool(comunication sucess or not
	bool MyRobot_ABS2Incre( int station_Num );
	//
	//change mode to V mode
	//>station number(0~5)
	//<bool(comunication sucess or not
	bool MyRobot_toVmode( int station_Num );
	//
	//change Incre encoder to ABS encoder
	//>station number(0~5)
	//<bool(comunication sucess or not
	bool MyRobot_Incre2ABS( int station_Num );
	//
	//change mode to P mode
	//>station number(0~5)
	//<bool(comunication sucess or not
	bool MyRobot_toPmode( int station_Num );
	//
	//change mode to Torque mode
	//>station number(0~5)
	//<bool(comunication sucess or not
	bool MyRobot_toTorqmode(int station_Num);
	//
	
	void MyRobot_CumulativeCmdPulse(int station_num,int &pulse);
	void MyRobot_Within1RevPosition(int station_num,int &pulse);
	void MyRobot_DroopPulse(int station_num,int &pulse);
	//read the ABS Position
	//>station number(0~5)
	//<pulse(the ABS Position value
	void MyRobot_ABS_Position(int station_num,int &pulse);
	//
private:
	//comunicate to the robot
	//>station number¡Acommand number¡Adata number¡Adata
	//<bool(comunication sucess or not
	bool MyRobot_Communicate( int station_num, char cmd_num1,char cmd_num2, char data_num1,char data_num2, uchar *data );
	//
	//if you want to write something to robot¡Ayou should set mode to basic setting parameter
	//>station number
	//<bool(comunication sucess or not
	bool MyRobot_Set2BasicSettingParameter(int station_Num);
	//
	//-------------------------------------------------------------------------------------
	//add checksum to the end of instruction
	//>instruction
	void getCheckSum(std::vector<uchar> *instruction);
	//
	//prodece instruction
	//>station number¡Acommand number¡Adata number
	//<instruction
	std::vector<uchar> produceInstruction(uchar stationNumber,uchar cmd1,uchar cmd2,uchar dataNO1,uchar dataNO2);
	//
	//prodece instruction
	//>station number¡Acommand number¡Adata number,data 
	//<instruction
	std::vector<uchar> produceInstruction(uchar stationNumber,uchar cmd1,uchar cmd2,uchar dataNO1,uchar dataNO2,uchar *data);
	//
	//print the characters which robot returns
	//>reply instruction
	void readCommand(std::vector<uchar> reply);
	//
	//change integer to char,for example,1->'1'
	//>integer
	//<char
	uchar int2char(int n);
	//
	//if the reply instruction is the group of Status Display Data¡Aread the value
	//>reply instruction
	//<the value 
	double returnStatusDisplayDataValue(std::vector<uchar> reply);
	//
	//if you want to get the ABS Position value¡Ayou should use this function to analyze the reply instruction
	//>reply instruction
	//<the value 
	double returnABSPositionDataValue(std::vector<uchar> reply);
	//
	//change hex to decimal,for example,'11'->17
	//>string
	//<double
	double hex2dec(char *str);
	//
	//change char to integer,for example,'3'->3,'A'->10
	//>char
	//<integer
	int char2int(uchar c);
	//

	
	

};
#endif