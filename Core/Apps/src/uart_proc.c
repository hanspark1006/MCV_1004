/*
 * uart_proc.c
 *
 *  Created on: Jan 5, 2024
 *      Author: catsa
 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <stdarg.h>
#include "main.h"
#include "stm32f1xx_it.h"
#include "apps.h"
#include "uart_proc.h"
#include "timer_ctrl.h"

#define _MODIFY_	0

int nLEDCount = 0;
uint8_t nRx1_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t nRx2_data[8] = {0,0,0,0,0,0,0,0};
uint8_t nRecv1_data = 0;
uint8_t nRecv2_data = 0;

#define RX_BUFFER_SIZE 512
unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char rear=0;
unsigned char front=0;
unsigned char nRecvData=0;

unsigned char rear2=0;
unsigned char front2=0;
unsigned char nRecvData2=0;

#define MAX_PRINT_BUF	0xFF
char debug_buf[MAX_PRINT_BUF];
void Printf(const char *fmt, ...)
{
#if 0
	va_list args = {0};

	va_start(args, fmt);
	vsnprintf(debug_buf, MAX_PRINT_BUF, fmt, args);
	va_end(args);

	HAL_UART_Transmit(&huart1, (uint8_t *)debug_buf, strlen(debug_buf), 0xFFFFFF);
#endif	
}

int _write(UART_HandleTypeDef huart, uint8_t* p, uint16_t len)
{
	HAL_UART_Transmit(&huart, p, len, 10);
	return len;
}

void DataSend1(int protocol, int data)
{
	char buff[12];
	uint8_t nData[8];

	sprintf(buff, "%07d", data);
	nData[0] = (uint8_t)protocol;
	for(int i=0, j=1; i < 7; i++, j++){
		nData[j] = (uint8_t)buff[i];
	}
	_write(huart1, nData, 8);
}

void DataSend2(int protocol, int data)
{
	char buff[12];
	uint8_t nData[8];

	sprintf(buff, "%07d", data);
	nData[0] = (uint8_t)protocol;
	for(int i=0, j=1; i < 7; i++, j++){
		nData[j] = (uint8_t)buff[i];
	}
	_write(huart2, nData, 8);
}

void DeviceData3Send(int data)
{
	int len = 0;
	char buff[24];

	sprintf(buff, ":%xR%03d\r\n", Device.nDeviceNo, data);
	for (int i = 0; i< 24; i++)
	{
		if (buff[i] == 0) break;
		len++;
	}
	_write(huart1, (uint8_t*)buff, len);
}

void DeviceData1Send(int data)
{
	int len = 0;
	char buff[24];
	sprintf(buff, ":%xR%d\r\n", Device.nDeviceNo, data);
	for (int i = 0; i< 24; i++)
	{
		if (buff[i] == 0) break;
		len++;
	}
	_write(huart1, (uint8_t*)buff, len);
}

unsigned char serial_data_put(unsigned char data)
{
	unsigned char index = (front + 1) % RX_BUFFER_SIZE;
	if(index == rear) return 0;
	front = index;
	rx_buffer[front] = data;
	return 1;
}

unsigned char serial_data_get(void)
{
	if(front == rear) return 0;
	rear = (rear + 1) % RX_BUFFER_SIZE;
	nRecvData =  rx_buffer[rear];
	return 1;
}

unsigned char serial_data2_put(unsigned char data)
{
	unsigned char index = (front2 + 1) % RX_BUFFER_SIZE;
	if(index == rear2) return 0;
	front2 = index;
	rx_buffer[front2] = data;
	return 1;
}

unsigned char serial_data2_get(void)
{
	if(front2 == rear2) return 0;
	rear2 = (rear2 + 1) % RX_BUFFER_SIZE;
	nRecvData2 =  rx_buffer[rear2];
	return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		serial_data_put(nRecv1_data);
		HAL_UART_Receive_IT(&huart1, &nRecv1_data, 1);
	}
	else if (huart->Instance == USART2)
	{
		serial_data2_put(nRecv2_data);
		HAL_UART_Receive_IT(&huart2, &nRecv2_data, 1);
	}
}

int num_data_modify(uint8_t *pRX_data)
{
	if ((Device.nDeviceNo == 1) && (*pRX_data >= 0x30) && (*pRX_data < 0x34)){
		*pRX_data -= 0x00; //????
	}else if ((Device.nDeviceNo == 2) && (*pRX_data >= 0x34) && (*pRX_data < 0x38)){
		*pRX_data -= 0x04;
	}else if ((Device.nDeviceNo == 3) && (*pRX_data >= 0x38) && (*pRX_data < 0x3C)){
		*pRX_data -= 0x08;
	}else if ((Device.nDeviceNo == 4) && (*pRX_data >= 0x3C) && (*pRX_data < 0x40)){
		*pRX_data -= 0x0C;
	}else{
		return 1;
	}

	return 0;
}

int bSerialOnOff[4] = {0, 0, 0, 0};
#if _MODIFY_
uint8_t proc_remote(void)
{
#if 1
	for(int i=0, j=1; i < 7; i++, j++){
		nRx1_data[i] = nRx1_data[j];
	}
	nRx1_data[7] = nRecvData;
#else
	nRx1_data[0] = nRx1_data[1];
	nRx1_data[1] = nRx1_data[2];
	nRx1_data[2] = nRx1_data[3];
	nRx1_data[3] = nRx1_data[4];
	nRx1_data[4] = nRx1_data[5];
	nRx1_data[5] = nRx1_data[6];
	nRx1_data[6] = nRx1_data[7];
	nRx1_data[7] = nRecvData;
#if 0
	Printf("->");
	for(int i=0; i < 8; i++){
		Printf("[%d:%x] ", i, nRx1_data[i]);
	}
	Printf("<-\r\n");
#endif
#endif

	if ((nRx1_data[4] == 0x02) && (nRx1_data[7] == 0x03)){
		if(num_data_modify(&nRx1_data[5])){
			return 1;
		}

		int idx = nRx1_data[5] - 0x30;
		if (nRx1_data[6] == 'o'){
			PWM_OUT(idx, Device.nSerialPWM[idx]);
			bSerialOnOff[idx] = 1;
		}else{
			PWM_OUT(idx, 0);
			bSerialOnOff[idx] = 0;
		}
	}else if ((nRx1_data[0] == 0x02) && (nRx1_data[7] == 0x03)){
		if(num_data_modify(&nRx1_data[1])){
			return 1;
		}

		int idx = nRx1_data[1] - 0x30;
		if (nRx1_data[2] == 'w'){
			int adc = 0;
			adc = (nRx1_data[3] - 0x30) * 1000;
			adc += ((nRx1_data[4] - 0x30) * 100);
			adc += ((nRx1_data[5] - 0x30) * 10);
			adc += (nRx1_data[6] - 0x30);
			adc /= 4;
			Device.nSerialPWM[idx] = adc;

			PWM_OUT(idx, Device.nSerialPWM[idx]);
		}
	}

	return 0;
}

void find_character(uint8_t RxData, int *arg)
{
	if (RxData == 'A') *arg = 10;
	else if (RxData == 'B') *arg = 11;
	else if (RxData == 'C') *arg = 12;
	else if (RxData == 'D') *arg = 13;
	else if (RxData == 'E') *arg = 14;
	else if (RxData == 'F') *arg = 15;
}

void proc_strobe(void)
{
	int i = 0;

	HAL_UART_Receive_IT(&huart1, &nRecv1_data, 1); //????
	for (i = 0; i < 23; i++){
		nRx1_data[i] = nRx1_data[i + 1];
	}

	nRx1_data[23] = nRecv1_data;
	if ((nRx1_data[22] == '\r') && (nRx1_data[23] == '\n')){
		int id = 0;
		int comm = 0;
		int type = 0;
		int channel = 0;
		int data = 0;
		for (i = 21; i >= 0; i--){
			if (nRx1_data[i] == ':') break;
		}

		if (i >= 0){
			id = nRx1_data[i + 1] - 0x30;
			if (id > 9){
				find_character(nRx1_data[i + 1], &id);
			}
			comm = nRx1_data[i + 2];
			type = nRx1_data[i + 3] - 0x30;
			if (type > 9){
				find_character(nRx1_data[i + 3], &type);
			}
			channel = nRx1_data[i + 4] - 0x30;
			if (channel > 9){
				find_character(nRx1_data[i + 4], &channel);
			}

			if (i == 16){
				data = (int)(nRx1_data[i + 5] - 0x30);
			}else if (i == 14){
				data = (int)(nRx1_data[i + 5] - 0x30) * 100;
				data += (int)(nRx1_data[i + 6] - 0x30) * 10;
				data += (int)(nRx1_data[i + 7] - 0x30);
			}

			if (comm == 'W'){
				if (type == 2){
					Device.nPULSE[channel - 1] = data;
					DataSend2(0xB0 + (channel - 1), data);
				}else if (type == 3){
					Device.nDELAY[channel - 1] = data;
					DataSend2(0xA0 + (channel - 1), data);
				}else if (type == 8){
					if (data == 1){
						if (Device.nDELAY[channel-1] == 0){
							//nPWMTimer[channel-1].nDelay = 0;
							nPWMTimer[channel-1] = 10000005;
						}else{
							//nPWMTimer[channel-1].nDelay = Device.nDELAY[channel-1];
							nPWMTimer[channel-1] = Device.nDELAY[channel-1] + 10000005;
						}
					}
				}else if (type == 10){
					Device.nEDGE[channel - 1] = data;
					DataSend2(0xC0 + (channel - 1), data);
				}else if ((type == 15) && (channel == 15)){
				// �����ϴ� �ڵ�

				}
			}else if (comm == 'R'){
				if (type == 2){
					DeviceData3Send(Device.nPULSE[channel-1]);
				}else if (type == 3){
					DeviceData3Send(Device.nDELAY[channel-1]);
				}else if (type == 10){
					DeviceData1Send(Device.nEDGE[channel-1]);
				}
			}
		}
		for (i = 0; i < 24; i++){
			nRx1_data[i] = 0;
		}
	}
}

void SerialRecvData(void)
{
	while (serial_data_get()){
		//Printf("cur status[%x]\r\n", Device.nDeviceCurrentStatus);
		if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_REMOTE){
			if(proc_remote()) continue;
		}else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_REMOTE){
			proc_strobe();
		}
	}
}
#else
void SerialRecvData(void)
{
	while (serial_data_get())
	{
		if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_REMOTE)
		{
			nRx1_data[0] = nRx1_data[1];
			nRx1_data[1] = nRx1_data[2];
			nRx1_data[2] = nRx1_data[3];
			nRx1_data[3] = nRx1_data[4];
			nRx1_data[4] = nRx1_data[5];
			nRx1_data[5] = nRx1_data[6];
			nRx1_data[6] = nRx1_data[7];
			nRx1_data[7] = nRecvData;

			if ((nRx1_data[4] == 0x02) && (nRx1_data[7] == 0x03))
			{
				if ((Device.nDeviceNo == 1) && (nRx1_data[5] >= 0x30) && (nRx1_data[5] < 0x34))
				{
					nRx1_data[5] -= 0x00;
				}
				else if ((Device.nDeviceNo == 2) && (nRx1_data[5] >= 0x34) && (nRx1_data[5] < 0x38))
				{
					nRx1_data[5] -= 0x04;
				}
				else if ((Device.nDeviceNo == 3) && (nRx1_data[5] >= 0x38) && (nRx1_data[5] < 0x3C))
				{
					nRx1_data[5] -= 0x08;
				}
				else if ((Device.nDeviceNo == 4) && (nRx1_data[5] >= 0x3C) && (nRx1_data[5] < 0x40))
				{
					nRx1_data[5] -= 0x0C;
				}
				else
				{
					continue;
				}
				int idx = nRx1_data[5] - 0x30;
				if (nRx1_data[6] == 'o')
				{
					PWM_OUT(idx, Device.nSerialPWM[idx]);
					bSerialOnOff[idx] = 1;
				}
				else
				{
					PWM_OUT(idx, 0);
					bSerialOnOff[idx] = 0;
				}
			}
			else if ((nRx1_data[0] == 0x02) && (nRx1_data[7] == 0x03))
			{
				if ((Device.nDeviceNo == 1) && (nRx1_data[1] >= 0x30) && (nRx1_data[1] < 0x34))
				{
					nRx1_data[1] -= 0x00;
				}
				else if ((Device.nDeviceNo == 2) && (nRx1_data[1] >= 0x34) && (nRx1_data[1] < 0x38))
				{
					nRx1_data[1] -= 0x04;
				}
				else if ((Device.nDeviceNo == 3) && (nRx1_data[1] >= 0x38) && (nRx1_data[1] < 0x3C))
				{
					nRx1_data[1] -= 0x08;
				}
				else if ((Device.nDeviceNo == 4) && (nRx1_data[1] >= 0x3C) && (nRx1_data[1] < 0x40))
				{
					nRx1_data[1] -= 0x0C;
				}
				else
				{
					continue;
				}
				int idx = nRx1_data[1] - 0x30;
				if (nRx1_data[2] == 'w')
				{
					int adc = 0;
					adc = (nRx1_data[3] - 0x30) * 1000;
					adc += ((nRx1_data[4] - 0x30) * 100);
					adc += ((nRx1_data[5] - 0x30) * 10);
					adc += (nRx1_data[6] - 0x30);
					adc /= 4;
					Device.nSerialPWM[idx] = adc;

					PWM_OUT(idx, Device.nSerialPWM[idx]);
				}
			}
		}
		else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_REMOTE)
		{
			int i = 0;
			HAL_UART_Receive_IT(&huart1, &nRecv1_data, 1);
			for (i = 0; i < 23; i++)
			{
				nRx1_data[i] = nRx1_data[i + 1];
			}
			nRx1_data[23] = nRecv1_data;
			if ((nRx1_data[22] == '\r') && (nRx1_data[23] == '\n'))
			{
				int id = 0;
				int comm = 0;
				int type = 0;
				int channel = 0;
				int data = 0;
				for (i = 21; i >= 0; i--)
				{
					if (nRx1_data[i] == ':') break;
				}
				if (i >= 0)
				{
					id = nRx1_data[i + 1] - 0x30;
					if (id > 9)
					{
						if (nRx1_data[i + 1] == 'A') id = 10;
						else if (nRx1_data[i + 1] == 'B') id = 11;
						else if (nRx1_data[i + 1] == 'C') id = 12;
						else if (nRx1_data[i + 1] == 'D') id = 13;
						else if (nRx1_data[i + 1] == 'E') id = 14;
						else if (nRx1_data[i + 1] == 'F') id = 15;
					}
					comm = nRx1_data[i + 2];
					type = nRx1_data[i + 3] - 0x30;
					if (type > 9)
					{
						if (nRx1_data[i + 3] == 'A') type = 10;
						else if (nRx1_data[i + 3] == 'B') type = 11;
						else if (nRx1_data[i + 3] == 'C') type = 12;
						else if (nRx1_data[i + 3] == 'D') type = 13;
						else if (nRx1_data[i + 3] == 'E') type = 14;
						else if (nRx1_data[i + 3] == 'F') type = 15;
					}
					channel = nRx1_data[i + 4] - 0x30;
					if (channel > 9)
					{
						if (nRx1_data[i + 4] == 'A') channel = 10;
						else if (nRx1_data[i + 4] == 'B') channel = 11;
						else if (nRx1_data[i + 4] == 'C') channel = 12;
						else if (nRx1_data[i + 4] == 'D') channel = 13;
						else if (nRx1_data[i + 4] == 'E') channel = 14;
						else if (nRx1_data[i + 4] == 'F') channel = 15;
					}
					if (i == 16)
					{
						data = (int)(nRx1_data[i + 5] - 0x30);
					}
					else if (i == 14)
					{
						data = (int)(nRx1_data[i + 5] - 0x30) * 100;
						data += (int)(nRx1_data[i + 6] - 0x30) * 10;
						data += (int)(nRx1_data[i + 7] - 0x30);
					}
					if (comm == 'W')
					{
						if (type == 2)
						{
							Device.nPULSE[channel - 1] = data;
							DataSend2(0xB0 + (channel - 1), data);
						}
						else if (type == 3)
						{
							Device.nDELAY[channel - 1] = data;
							DataSend2(0xA0 + (channel - 1), data);
						}
						else if (type == 8)
						{
							if (data == 1)
							{
								if (Device.nDELAY[channel-1] == 0)
								{
									nPWMTimer[channel-1] = 10000001;
								}
								else
								{
									nPWMTimer[channel-1] = Device.nDELAY[channel-1] + 10000000;
								}
							}
						}
						else if (type == 10)
						{
							Device.nEDGE[channel - 1] = data;
							DataSend2(0xC0 + (channel - 1), data);
						}
						else if ((type == 15) && (channel == 15))
						{
						// ?�?�하??코드

						}
					}
					else if (comm == 'R')
					{
						if (type == 2)
						{
							DeviceData3Send(Device.nPULSE[channel-1]);
						}
						else if (type == 3)
						{
							DeviceData3Send(Device.nDELAY[channel-1]);
						}
						else if (type == 10)
						{
							DeviceData1Send(Device.nEDGE[channel-1]);
						}
					}
				}
				for (i = 0; i < 24; i++)
				{
					nRx1_data[i] = 0;
				}
			}
		}
	}
}
#endif
int nChangeFlag = 0;
void SerialRecvData2(void)
{
	uint16_t pin[]={EX_INPUT1_Pin, EX_INPUT2_Pin, EX_INPUT3_Pin, EX_INPUT4_Pin};
	uint8_t	ch_idx;

	while (serial_data2_get())
	{
		for(int i=0, j=1; i < 7; i++, j++){
			nRx2_data[i] = nRx2_data[j];
		}
		nRx2_data[7] = nRecvData2;

		//Printf("nRx2_data[0] %x\r\n", nRx2_data[0]);
		if (nRx2_data[0] >= 0x80){
			char recv_data[10];
			unsigned char protocol = nRx2_data[0];
			unsigned char channel = nRx2_data[1];

//			for(int cnt = 0; cnt < 8; cnt++)
//			{
//				Printf("[%x] ", nRx2_data[cnt]);
//			}
//			Printf("\r\n");

			for(int i=0, j=2; i < 6; i++,j++){
				recv_data[i]=nRx2_data[j];
			}
			recv_data[6]=0;

			int data = atoi(recv_data);
			if (protocol == DEVICE_STATUS)
			{
				Device.nDeviceCurrentStatus = data;

				if (Device.nDeviceCurrentStatus != Device.nDeviceCurrentStatus_old){
					Device.nDeviceCurrentStatus_old = Device.nDeviceCurrentStatus;
					setCurDeviceStatus();
					nChangeFlag = 1;
				}

				if (Device.nDeviceNo == 1){
					DataSend2(DEVICE_ECHO, DEVICE_FIRMWARE_VERSION);
				}

				if ((Device.nDeviceCurrentStatus >= LCD_STATUS_MODE_LOCAL) && (Device.nDeviceCurrentStatus <= LCD_STATUS_MODE_EXT))	{
					if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_LOCAL){
						for(int ch = 0; ch < MAX_PWM_CH; ch++){
							PWM_OUT(ch, Device.nPWM[ch]);
						}
					}else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_REMOTE){
						if (nChangeFlag == 1){
							for(int ch = 0; ch < MAX_PWM_CH; ch++){
								PWM_OUT(ch, 0);
								bSerialOnOff[ch] = 0;
							}
						}
					}else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_EXT){
						if (nChangeFlag == 1){
							for(int input = 0; input < MAX_SW_PORT; input++){
								if(bEX_INPUT[input] == 0) PWM_OUT(input, Device.nPWM[input]);
								else PWM_OUT(input, 0);
							}
						}
					}
					nChangeFlag = 0;
				}else{ // STROBE
					for(int cnt = 0; cnt < MAX_SW_PORT; cnt++){
						PWM_OUT(cnt, 0);
					}
#if DAEGON_MODEL
					//Printf("Daegon PWM out!!\r\n");
#else
					pwm_timer_disable();
					strobe_timer_init();
#endif
				}
			}else if (protocol == STROBE_DELAY){
				if ((channel >= 0x30) && (channel <= 0x33)) Device.nDELAY[channel-0x30]=data;
			}else if (protocol == STROBE_PULSE){
				if ((channel >= 0x30) && (channel <=0x33)) Device.nPULSE[channel-0x30]=data;
			}else if (protocol == STROBE_RISEFALL){
				GPIO_InitTypeDef GPIO_InitStruct = {0};
				if ((channel >= 0x30) && (channel <= 0x33)){
					ch_idx = channel - 0x30;
					Device.nEDGE[ch_idx]=data;
					GPIO_InitStruct.Pin = pin[ch_idx];
					if (Device.nEDGE[ch_idx] == 0)
					{
						GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
					}
					else if (Device.nEDGE[ch_idx] == 1)
					{
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
					}
					else
					{
						GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
					}
					GPIO_InitStruct.Pull = GPIO_PULLUP;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				}
			}
			else if (protocol == CV_PWM)
			{
#ifdef TEST_10KHZ
//				if(data > 0){
//					data-=5;
//				}
#else
				data *= 100;
				data = ((data * 255) / 100) / 100;
#endif
				uint8_t ch_base=0x30;
				if (Device.nDeviceNo >= 1 && Device.nDeviceNo <= 4 ){
					ch_base += (Device.nDeviceNo-1)*4;
					ch_idx = channel - ch_base;
					Device.nPWM[ch_idx]=data;
				}
			}
			else if (protocol == DEVICE_UPDATE)
			{
				if (data == 1) JumpToBootloader();
			}
		}
	}
}

void Serial_ResetRequest(void)
{
	DataSend2(DEVICE_RESET, DEVICE_RESET_REQUEST);
}

void uart_proc_init(void)
{
	HAL_UART_Receive_IT(&huart1, &nRecv1_data, 1);
	HAL_UART_Receive_IT(&huart2, &nRecv2_data, 1);
}
