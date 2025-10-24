// bluetooth.h
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx_hal.h"

void Bluetooth_Init(UART_HandleTypeDef* huart);
void Bluetooth_RxCpltCallback(UART_HandleTypeDef* huart);
void SendEncoderData(void);

// 里程计相关
void Bluetooth_SendPose(void);
void Bluetooth_ProcessRadarCommands(void);

// 雷达数据发送
void SendRadarData(void);

#endif
