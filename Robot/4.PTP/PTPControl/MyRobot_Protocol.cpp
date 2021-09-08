#include "stdafx.h"
#define NOMINMAX
#include "MyRobot_Protocol.h"

//
//Class BufferedAsyncSerial
//

MyRobot_Protocol::MyRobot_Protocol(): MyRobot_AsyncSerial()
{
    setReadCallback(boost::bind(&MyRobot_Protocol::Protocol_readCallback, this, _1, _2));
}

MyRobot_Protocol::MyRobot_Protocol(const std::string& devname,
        unsigned int baud_rate,
        boost::asio::serial_port_base::parity opt_parity,
        boost::asio::serial_port_base::character_size opt_csize,
        boost::asio::serial_port_base::flow_control opt_flow,
        boost::asio::serial_port_base::stop_bits opt_stop)
        :MyRobot_AsyncSerial(devname,baud_rate,opt_parity,opt_csize,opt_flow,opt_stop)
{
    setReadCallback(boost::bind(&MyRobot_Protocol::Protocol_readCallback, this, _1, _2));
}

size_t MyRobot_Protocol::Protocol_binaryRead(char *data, size_t size)
{
    boost::lock_guard<boost::mutex> l(Protocol_readQueueMutex);
	size_t result=std::min(size,Protocol_readQueue.size());//check
    std::vector<char>::iterator it=Protocol_readQueue.begin()+result;
    copy(Protocol_readQueue.begin(),it,data);
    Protocol_readQueue.erase(Protocol_readQueue.begin(),it);
    return result;
}

std::vector<char> MyRobot_Protocol::Protocol_binaryRead()
{
    boost::lock_guard<boost::mutex> l(Protocol_readQueueMutex);
    std::vector<char> result;
    result.swap(Protocol_readQueue);
    return result;
}

std::string MyRobot_Protocol::Protocol_readString()
{
    boost::lock_guard<boost::mutex> l(Protocol_readQueueMutex);
    std::string result(Protocol_readQueue.begin(),Protocol_readQueue.end());
    Protocol_readQueue.clear();
    return result;
}

std::string MyRobot_Protocol::Protocol_readString(const std::string delim)
{
    boost::lock_guard<boost::mutex> l(Protocol_readQueueMutex);
    std::vector<char>::iterator it=Protocol_findStringInVector(Protocol_readQueue,delim);
    if(it==Protocol_readQueue.end()) 
		return "";
    std::string result(Protocol_readQueue.begin(),it);
    it+=delim.size();//Do remove the delimiter from the queue
    Protocol_readQueue.erase(Protocol_readQueue.begin(),it);
    return result;
}

void MyRobot_Protocol::Protocol_readCallback(const char *data, size_t len)
{
    boost::lock_guard<boost::mutex> l(Protocol_readQueueMutex);
    Protocol_readQueue.insert(Protocol_readQueue.end(),data,data+len);
}

std::vector<char>::iterator MyRobot_Protocol::Protocol_findStringInVector(
        std::vector<char>& v,const std::string& s)
{
    if(s.size()==0) return v.end();

    std::vector<char>::iterator it=v.begin();
    for(;;)
    {
        std::vector<char>::iterator result=find(it,v.end(),s[0]);
        if(result==v.end()) return v.end();//If not found return

        for(size_t i=0;i<s.size();i++)
        {
            std::vector<char>::iterator temp=result+i;
            if(temp==v.end()) return v.end();
            if(s[i]!=*temp) goto mismatch;
        }
        //Found
        return result;

        mismatch:
        it=result+1;
    }
}

WORD MyRobot_Protocol::Protocol_crc16(unsigned char *nData, WORD wLength)
{	
	int j;
	unsigned int reg_crc=0xFFFF;

	while(wLength--)
	{
		unsigned char temp=*nData++;
		reg_crc ^= temp;
		
		for(j=0;j<8;j++)
		{
			if(reg_crc & 0x01) /* LSB(b0)=1 */
			{	
				reg_crc=reg_crc>>1;
				reg_crc=(reg_crc) ^ 0xA001;
			}
			else
			{	
				reg_crc=reg_crc>>1;
			}
		}

	 }

	return reg_crc;
}
//
MyRobot_Protocol::errBuffer MyRobot_Protocol::Protocol_crcCheck( std::vector<uchar> data, const int start, const int end)
{
	//check if start smaller than 1
	try
	{
		if(start < 1)
			throw "start can not be 1...";
	}
	catch(const char* msg)
	{
		std::cout << msg << std::endl;
	}

	//declare
	uchar* tempMatrix=NULL;
	std::vector<uchar> tempBuffer;
	errBuffer tempcrc;
	
	//copy data to dataBuffer                //howard:用for loop來做比使用assign還來的清楚
	for(int i=start-1;i<=end-1;i++)
	{
		tempBuffer.push_back(data[i]);
	}
	

	tempMatrix=&tempBuffer[0];

	//crc check
	WORD crcReturn=this->Protocol_crc16(tempMatrix, tempBuffer.size());
	tempcrc.errHigh=crcReturn & CRC_HIGH;
	tempcrc.errLow=_SHIFT_8( (crcReturn & CRC_LOW) );


	return tempcrc;
}
//howard
MyRobot_Protocol::errBuffer MyRobot_Protocol::Protocol_mitsubishiCheck( std::vector<uchar> data, const int start, const int end)
{
	errBuffer temperr;
	//---------------------------------------------------------------------
	
	int total=0;
	for(int i=start-1/*howard:這裡的start和end並非vector的index,而是vector的第幾個元素,和crccheck一樣,所以要減1*/;i<=end-1;i++)
	{
		total+=data[i];
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
	
	temperr.errHigh=checkSumForChar[0];
	temperr.errLow=checkSumForChar[1];
	/*data.push_back(checkSumForChar[0]);
	data.push_back(checkSumForChar[1]);*/
	//------------------------------------------------------------------------
	return temperr;
}
//
bool MyRobot_Protocol::Protocol_communicate( std::vector<uchar> &Data, const int type )
{
	//check vector, if the vector is empty throw warning message
	try
	{
		if(Data.empty())
		{
			throw "Send data is empty...";
		}
	}
	catch(const char* msg)
	{
		std::cout << msg << std::endl;
		return false;
	}

	//declare variable
	int size=Data.size();
	errBuffer temperr;

	switch(type)
	{
		case 1:	//modbus protocol			
			//execute crc check
			temperr=this->Protocol_crcCheck(Data,1, size);
		break;

		case 2: //mitsubishi driver protocol			
			temperr=this->Protocol_mitsubishiCheck(Data,2/*howard:表示地2個元素，因為第一個元素不算在checksum中*/, size);		
		break;
	}	
	
	//push CRC check in the end of original data
	Data.push_back(temperr.errHigh);
	Data.push_back(temperr.errLow);	

	std::cout << "print send data..." << std::endl;

	for(int i=0; i < Data.size(); i++)
	{
		std::cout << std::hex<<(int)Data.at(i) << ' ';
	}
	std::cout<<std::endl;

	//write data into serial port
	uchar* dataMatrix=&Data[0];
	this->write((char*) dataMatrix,size+2);

	//clear sendData
	Data.clear();

	/*read data, if reading times is over TEST_NUM(ref:MyRobot_ProtocolDef.h)
	the reading is failed!
	*/
	for(int i=1; i<=/*howard:這裡要改成<=，不燃程式重複市的次數會比TEST_NUM小一次*/TEST_NUM; i++)
	{
		//wait for reading
		Sleep(WAIT_READ);

		std::vector<char> cvector=(this->Protocol_binaryRead());

		//if the return is not empty, then return the data
		if(!cvector.empty())
		{
			int cvec_size=cvector.size();
			

			std::cout << "print return data..." << std::endl;

			for(int i=0; i < cvec_size; i++)
			{
				Data.push_back(cvector[i]);
				std::cout <<std::hex<<(int) Data.at(i) << ' ';
			}
			std::cout<<std::endl;
			break;
		}
		else
		{
			std::cout << "\n" << i << " Reading fails" << ' ';
			
			//return false;//howard:此行可刪去，若加這行的話，程式若是第一次沒讀到值就會回傳false
			
		}
		//------------------------howard:如果是過TEST_NUM次,cvector還是沒有東西，就要回傳false
		if(i==TEST_NUM)
		{
			return false;
		}
		//----------------------------------------------------------------
	}

	//Return data error check
	switch(type)
	{
		case 1:	//modbus protocol			
			//execute crc check
			temperr=this->Protocol_crcCheck(Data,1, Data.size()-2);
		break;

		case 2: //mitsubishi driver protocol			
			temperr=this->Protocol_mitsubishiCheck(Data,2/*howard:三菱的checksum是從第二個開始算*/, Data.size()-2);		
		break;
	}

	if(temperr.errHigh != Data.at(Data.size()-2) && temperr.errLow != Data.at(Data.size()-1))
	{
		std::cout << "\n return data crc check fail!" << ' '<<std::endl;
		return false;
	}
	else
		return true;
	
}

MyRobot_Protocol::~MyRobot_Protocol()
{
	this->clearReadCallback();
}
