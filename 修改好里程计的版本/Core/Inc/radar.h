#ifndef __RADAR_H__
#define __RADAR_H__

#include "main.h"
#include "usart.h"
#include "gpio.h"

//雷达帧头定义
#define LS_HEADER1      0x0A   //第一个帧头
#define LS_HEADER2      0x05   //第二个帧头
#define LS_F_LEN        84   //雷达帧长度

// EXPRESS_SCAN命令相关定义
#define EXPRESS_SCAN_CMD    0x82  // EXPRESS_SCAN命令
#define EXPRESS_SCAN_LEN    0x05  // EXPRESS_SCAN负载数据长度
#define EXPRESS_SCAN_CHECK  0x22  // EXPRESS_SCAN校验码

//雷达结构体定义
typedef struct
{
	uint16_t       angle;     //角度
	uint16_t    distance;     //距离
}LaserPointTypeDef;

void LS_DataHandle(void);
void AX_LASER_Start(void);
void AX_LASER_Stop(void);
void AX_LASER_Test(void);
void AX_LASER_GetData(void);

#endif
