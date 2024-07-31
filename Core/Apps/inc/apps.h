/*
 * uart_serial.h
 *
 *  Created on: Jan 5, 2024
 *      Author: catsa
 */

#ifndef INC_UART_SERIAL_H_
#define INC_UART_SERIAL_H_

#define TIMER_UNIT		1  // 10us
#define INIT_DELAY_MS	10000000
#define MAX_PLUSE_DELAY	1000

#define MAX_SW_PORT	4
#define MAX_PWM_CH	4

#define STROBE_DELAY					0xC1
#define STROBE_PULSE					0xC2
#define STROBE_RISEFALL					0xC3
#define CV_PWM							0xC4
#define DEVICE_STATUS					0xD1
#define DEVICE_ECHO						0xE0
#define SENSING_LED						0xE1
#define DEVICE_RESET					0xE2
#define DEVICE_UPDATE					0xF0

#define DEVICE_FIRMWARE_VERSION			114
#define DEVICE_RESET_REQUEST			115

#define LCD_STATUS_INITIALIZING			30
#define LCD_STATUS_MODE_LOCAL			31
#define LCD_STATUS_MODE_REMOTE			32
#define LCD_STATUS_MODE_EXT				33

#define LCD_STATUS_MODE_STROBE_NORMAL	41
#define LCD_STATUS_MODE_STROBE_REMOTE	42
#define LCD_STATUS_MODE_STROBE_TEST		43
#define LCD_STATUS_SET_TIME_DELAY		44
#define LCD_STATUS_SET_TIME_PULSE		45
#define LCD_STATUS_SET_INPUT			46
#define LCD_STATUS_FIRMWARE_UPDATE		47

typedef struct
{
	int nDeviceNo;
	int nDeviceCurrentStatus;
	int nDeviceCurrentStatus_old;
	int nPWM[4];
	int nDELAY[4];
	int nPULSE[4];
	int nEDGE[4];
	int nSerialPWM[4];
}DEVICE;

extern uint8_t bSW[];
extern uint8_t bEX_INPUT[];
extern DEVICE Device;

void apps_dev_init(void);
void apps_proc(void);
#endif /* INC_UART_SERIAL_H_ */
