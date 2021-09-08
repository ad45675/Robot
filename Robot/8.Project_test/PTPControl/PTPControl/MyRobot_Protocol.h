/*
*	The main function of this class is providing serial port communication, including CRC check function
*	Author: Wythe, howard
*	Version: v1.4
*	History: 2014/02/07 firs version
*			 2014/03/20 add Protocol_mitsubishiCheck function
*			 2014/04/29 add the ifdef to protect this library
*            2014/06/24 compensate something and bug
*/

#include "MyRobot_AsyncSerial.h"
#include "MyRobot_ProtocolDef.h"
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>
#include <typeinfo>

#ifndef MYROBOT_PROTOCOL_H
#define	MYROBOT_PROTOCOL_H

typedef unsigned char uchar;

class MyRobot_Protocol: public MyRobot_AsyncSerial
{
public:
    MyRobot_Protocol();

    /**
    * Opens a serial device.
    * \param1 devname serial device name, example "/dev/ttyS0" or "COM1"
    * \param2 baud_rate serial baud rate
    * \param3 opt_parity serial parity, default none
    * \param4 opt_csize serial character size, default 8bit
    * \param5 opt_flow serial flow control, default none
    * \param6 opt_stop serial stop bits, default 1
    * \throws boost::system::system_error if cannot open the serial device
    * 
    */
    MyRobot_Protocol(const std::string& devname, unsigned int baud_rate,
        boost::asio::serial_port_base::parity opt_parity=
            boost::asio::serial_port_base::parity(
			boost::asio::serial_port_base::parity::even),
        boost::asio::serial_port_base::character_size opt_csize=
            boost::asio::serial_port_base::character_size(8),
        boost::asio::serial_port_base::flow_control opt_flow=
            boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none),
        boost::asio::serial_port_base::stop_bits opt_stop=
            boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));

    /**
     * Read some data asynchronously. Returns immediately.
     * \param1 data array of char to be read through the serial device
     * \param2 size array size
     * \return numbr of character actually read 0<=return<=size
     */
    size_t Protocol_binaryRead(char *data, size_t size);

    /**
     * Read all available data asynchronously. Returns immediately.
     * \return the receive buffer. It is empty if no data is available
     */
    std::vector<char> Protocol_binaryRead();

    /**
     * Read a string asynchronously. Returns immediately.
     * Can only be used if the user is sure that the serial device will not
     * send binary data. For binary data read, use read()
     * The returned string is empty if no data has arrived
     * \return a string with the received data.
     */
    std::string Protocol_readString();

     /**
     * Read a line asynchronously. Returns immediately.
     * Can only be used if the user is sure that the serial device will not
     * send binary data. For binary data read, use read()
     * The returned string is empty if the line delimiter has not yet arrived.
     * \param delimiter line delimiter, default='\n'
     * \return a string with the received data. The delimiter is removed from
     * the string.
     */
    std::string Protocol_readString(const std::string delim="\n");

    virtual ~MyRobot_Protocol();

private:

    /**
     * Read callback, stores data in the buffer
     */
    void Protocol_readCallback(const char *data, size_t len);

    /**
     * Finds a substring in a vector of char. Used to look for the delimiter.
     * \param1 v vector where to find the string
     * \param2 s string to find
     * \return the beginning of the place in the vector where the first
     * occurrence of the string is, or v.end() if the string was not found
     */
    static std::vector<char>::iterator Protocol_findStringInVector(std::vector<char>& v,
            const std::string& s);

    std::vector<char> Protocol_readQueue;
    boost::mutex Protocol_readQueueMutex;

public:
	/*
	* Function:Modbus CRC 16 method
	* Input: procotol data, total length
	* Output: CRC in Word
	*/
	WORD Protocol_crc16(uchar *nData, WORD wLength);
	

	/*
	* Function: Modbus CRC check practice
	* Input: data, checking start pos(i=1), checking end pos
	* Output: errBuffer struct
	*/
	class errBuffer
	{
	public:
		uchar errHigh;
		uchar errLow;
	};
	errBuffer Protocol_crcCheck( std::vector<uchar> data, const int start, const int end );
	
	/*
	* Function: mitsubishi dirver check protocol function
	* Input: data, checking start pos(i=1), checking end pos
	* Output: errBuffer struct 
	*/
	errBuffer Protocol_mitsubishiCheck( std::vector<uchar> data, const int start, const int end );
	
	

	/*
	* Function: Send package to protocol
	* Input: Protocol data and Protocol type
	* Output: status
	*/
	bool Protocol_communicate( std::vector<uchar> &Data, const int type );

};

#endif //MYROBOT_PROTOCOL_H