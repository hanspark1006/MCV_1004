/*
 * uart_proc.h
 *
 *  Created on: Jan 5, 2024
 *      Author: catsa
 */

#ifndef APPS_INC_UART_PROC_H_
#define APPS_INC_UART_PROC_H_

void Printf(const char *fmt, ...);

void DataSend1(int protocol, int data);
void DataSend2(int protocol, int data);
void DeviceData3Send(int data);
void DeviceData1Send(int data);

void SerialRecvData(void);
void SerialRecvData2(void);
void Serial_ResetRequest(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void uart_proc_init(void);
#endif /* APPS_INC_UART_PROC_H_ */
