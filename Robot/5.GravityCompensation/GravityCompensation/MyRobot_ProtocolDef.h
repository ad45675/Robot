#ifndef MYROBOT_PROTOCOLDEF_H
#define	MYROBOT_PROTOCOLDEF_H

//Protocol Type
#define MODBUS 1
#define MUSTUBISHI_DRIVER 2

//Read Data test times
#define TEST_NUM 3

//macro
#define CRC_HIGH 0x00FF
#define CRC_LOW 0xFF00
#define _SHIFT_8(x) (x>>8)

//wait reading(ms)
#define WAIT_READ 100

#endif