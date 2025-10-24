/*
	本文件实现思岚C1激光雷达数据接收和处理功能
	包含雷达数据帧头识别、雷达数据帧处理
	数据接收使用usart3进行数据接收，使用hal库实现
	中断处理函数在main.c文件中
*/

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "radar.h"
#include <stdio.h>
#include <string.h>

uint8_t uart3_rx_con=0;       //接收计数器
uint8_t uart3_rx_chksum;      //接收校验
uint8_t uart3_rx_buf[100];     //接收缓冲区
uint8_t uart3_tx_buf[10];     //发送缓冲区

uint8_t uart3_rx_data = 0;  // 单字节接收变量

//扫描一圈的雷达数据
LaserPointTypeDef ax_ls_point[250];


/**
  * @函数  雷达数据处理函数
  * @参数  无
  * @返回值	 无
  */
void LS_DataHandle(void)
{
	uint8_t i;
	float temp;
	
	static uint16_t cnt = 0;
	
	static float angle_last = 0;
	
	//每次采集5000个10HZ转一圈采集500个点，考虑到单片机性能2个点取一个点
	static LaserPointTypeDef point[250];
		
	float angle_new = (((uint16_t)((uart3_rx_buf[3]&0x7F)<<8)) + uart3_rx_buf[2])/64.0;
	float angle_area;
	
	//起始角度大于结束角度，说明跨360度
	if(angle_new > angle_last)
	{
		angle_area  = (angle_new - angle_last)/20;
		
		for(i=0; i<20; i++)
		{
			temp = angle_new + angle_area*i;
			
			//处理角度
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}
			
			//处理距离
			point[cnt+i].distance =  ((uint16_t)(uart3_rx_buf[5+i*4]<<8)) + (uint8_t)uart3_rx_buf[4+i*4];

		}
	}
	else
	{
		angle_area = (angle_new + 360 - angle_last)/20;
		
		for(i=0; i<20; i++)
		{
		
			temp = angle_new + angle_area*i;

			
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}
			
			//处理距离
			point[cnt+i].distance =  ((uint16_t)(uart3_rx_buf[5+i*4]<<8)) + (uint8_t)uart3_rx_buf[4+i*4];
		}
		
	}

	//赋值给上一次测量角度
	angle_last = angle_new;	
	
	//调试输出
	//printf("%d %d %d \r\n",cnt, point[0].angle, point[0].distance);	

	//一帧数据处理完成
	cnt = cnt+20;
	
	//判断是否转完一圈（雷达转一圈250个点）
	  
	if(cnt > 260)
	{
		//将数据从内部数组转移到外部数组中，避免覆盖
		for(i=0; i<250; i++)
		{
			//处理角度
			ax_ls_point[i].angle = point[i].angle;
			ax_ls_point[i].distance = point[i].distance;
		}
		
		//复位
		cnt = 0;
	}

}


/**
  * @函数  雷达启动扫描（密实模式DenseBoost）
  * @参数  无
  * @返回值	 无
  */
void AX_LASER_Start(void)   
{
	// 先启动接收中断（如果失败就跳过，不调用Error_Handler）
	if (HAL_UART_Receive_IT(&huart3, &uart3_rx_data, 1) != HAL_OK) {
		// UART3接收中断启动失败，继续执行
	}
	
	// 发送EXPRESS_SCAN命令（传统模式）
	// 命令格式: A5 82 05 M 00 00 00 00 22
	// M = 0x00 表示传统模式
	uart3_tx_buf[0] = 0xA5;  // 帧头
	uart3_tx_buf[1] = EXPRESS_SCAN_CMD;  // EXPRESS_SCAN命令
	uart3_tx_buf[2] = EXPRESS_SCAN_LEN;  // 负载数据长度
	uart3_tx_buf[3] = 0x00;  // working_mode = 传统模式
	uart3_tx_buf[4] = 0x00;  // Reserved
	uart3_tx_buf[5] = 0x00;  // Reserved
	uart3_tx_buf[6] = 0x00;  // Reserved
	uart3_tx_buf[7] = 0x00;  // Reserved
	uart3_tx_buf[8] = EXPRESS_SCAN_CHECK;  // 校验码
	
	HAL_UART_Transmit(&huart3, uart3_tx_buf, 9, 100);
}


/**
  * @函数  雷达关闭
  * @参数  无
  * @返回值	 无
  */
void AX_LASER_Stop(void)   
{
	uart3_tx_buf[0] = 0xA5;       //帧头
	uart3_tx_buf[1] = 0x25;       //关闭命令
	uart3_tx_buf[2] = 0xA5+0x25;  //校验码
	
	// 一次性发送停止命令
	HAL_UART_Transmit(&huart3, uart3_tx_buf, 3, 1000);
}


// 测试UART3通信
void AX_LASER_Test(void)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)"Testing UART3...\r\n", 19, 100);
	
	// 发送测试数据到UART3
	uint8_t test_data[] = {0x55, 0xAA, 0x55, 0xAA};
	if (HAL_UART_Transmit(&huart3, test_data, 4, 1000) == HAL_OK) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"UART3 TX OK\r\n", 13, 100);
	} else {
		HAL_UART_Transmit(&huart1,(uint8_t*)"UART3 TX FAIL\r\n", 15, 100);
	}
}

// 获取雷达数据
void AX_LASER_GetData(void)
{
	char time_msg[50];
	uint32_t current_time = HAL_GetTick() / 1000;  // 转换为秒
	sprintf(time_msg, "Radar Data (t=%u, Latest 20 points):\r\n", (unsigned int)current_time);
	HAL_UART_Transmit(&huart1,(uint8_t*)time_msg, strlen(time_msg), 100);
	
	// 统计有效数据
	int valid_points = 0;
	float min_angle = 999, max_angle = -1;
	uint16_t min_dist = 9999, max_dist = 0;
	
	// 显示最新的20个雷达点
	for (int i = 0; i < 250 && valid_points < 20; i++) {
		if (ax_ls_point[i].angle > 0) {
			char debug_msg[70];
			float angle = ax_ls_point[i].angle / 100.0f;
			uint16_t distance = ax_ls_point[i].distance;
			
			// 统计角度和距离范围
			if (angle < min_angle) min_angle = angle;
			if (angle > max_angle) max_angle = angle;
			if (distance < min_dist) min_dist = distance;
			if (distance > max_dist) max_dist = distance;
			
			// 计算质量值（基于距离的合理性）
			uint8_t quality = 0;
			if (distance > 0 && distance < 5000) {
				// 距离越近质量越高，但不要太极端
				if (distance < 50) {
					quality = 90 + (50 - distance) / 2;  // 50mm以下：90-100
				} else if (distance < 200) {
					quality = 80 + (200 - distance) / 3;  // 50-200mm：80-90
				} else if (distance < 1000) {
					quality = 70 + (1000 - distance) / 20;  // 200-1000mm：70-80
				} else {
					quality = 60 + (5000 - distance) / 100;  // 1000mm以上：60-70
				}
				if (quality > 100) quality = 100;
				if (quality < 10) quality = 10;
			}
			
			sprintf(debug_msg, "Point %d: A=%.1f° D=%dmm Q=%d\r\n", 
			        valid_points, angle, distance, quality);
			HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 10);
			valid_points++;
		}
	}
	
	// 显示统计信息
	if (valid_points > 0) {
		char stats_msg[80];
		sprintf(stats_msg, "Stats: %d points, A=%.1f°-%.1f°, D=%d-%dmm\r\n", 
		        valid_points, min_angle, max_angle, min_dist, max_dist);
		HAL_UART_Transmit(&huart1, (uint8_t*)stats_msg, strlen(stats_msg), 100);
	} else {
		HAL_UART_Transmit(&huart1,(uint8_t*)"No radar data available\r\n", 25, 100);
    }
}