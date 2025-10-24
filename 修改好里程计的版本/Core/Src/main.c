/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "bluetooth.h"
#include "encoder.h"
#include "mpu6500.h"
#include "odometry.h"
#include "radar.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// car_command is accessed from both main loop and UART interrupt
// volatile ensures proper memory access, uint8_t read/write is atomic on ARM Cortex-M
volatile uint8_t car_command = 1; // 初始停止
extern LaserPointTypeDef ax_ls_point[250];	//雷达点云数组（结构体数组，包含角度、距离）
extern uint8_t uart3_rx_con;       //接收计数器
extern uint8_t uart3_rx_chksum;      //接收校验
extern uint8_t uart3_rx_buf[100];     //接收缓冲区
extern uint8_t uart3_rx_data;  // 单字节接收变量
MPU6500_Data mpu_raw, mpu_calib;
MPU6500_Calib mpu_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 初始化
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	// Initialize motor control system
	Car_Init();
  Bluetooth_Init(&huart1); // 初始化蓝牙模块，使用USART1
  car_command = 1; // 默认停止
	Encoder_Init();

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
	    Error_Handler(); // 定时器启动失败
	}

  AX_LASER_Start();	// 发送请求报文，雷达开始发送应答报文

	if (MPU6500_Init(&hi2c1) != 0) {
	    // MPU6500初始化失败，可以选择继续运行或停止
	    // 这里选择继续运行，但不使用陀螺仪数据
	} else {
	    MPU6500_Calibrate(&hi2c1, &mpu_ctx); // 陀螺仪校准
	}

  Odometry_Init(); // 里程计初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  // 速度采样相关静态变量
  //static uint32_t last_tick = 0;
  //static int32_t last_count_left = 0;
  //static int32_t last_count_right = 0;
  //last_count_left = __HAL_TIM_GET_COUNTER(&htim2);
  //last_count_right = __HAL_TIM_GET_COUNTER(&htim4);
  //last_tick = HAL_GetTick();
	uint32_t last_odom_tick = HAL_GetTick();
  uint32_t last_btsend_tick = HAL_GetTick();
  uint32_t last_pid_tick = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			  // --- 每10ms 更新里程计 ---
    // 注意：Encoder_Update() 已经在 TIM7 中断中调用（精确10ms），这里不需要重复调用
    if (HAL_GetTick() - last_odom_tick >= 10)
    {
        // IMU更新
        if (MPU6500_Read_All(&hi2c1, &mpu_raw) == 0) {
            MPU6500_GetCalibratedData(&mpu_raw, &mpu_ctx, &mpu_calib);
            MPU6500_Convert_Unit(&mpu_calib);
            MPU6500_LowPassFilter(&mpu_calib, &mpu_ctx, 0.2f);
        } else {
            // MPU6500读取失败，使用零角速度避免里程计漂移
            mpu_ctx.gyro_z_filt = 0.0f;
        }

        // Encoder_Update() 已在 TIM7 中断中调用，此处删除重复调用

        Odometry_Update(
            mpu_ctx.gyro_z_filt,
            left_speed_mps,
            right_speed_mps
        );

        last_odom_tick = HAL_GetTick();
    }

    // --- 每100ms 蓝牙发送位姿 ---
    if (HAL_GetTick() - last_btsend_tick >= 1000)
    {
        Bluetooth_SendPose();
        last_btsend_tick = HAL_GetTick();
    }
    
    // 处理雷达命令（非阻塞）
    Bluetooth_ProcessRadarCommands();
    
    // 原有蓝牙 + 控车逻辑不动
		
    switch(car_command) {
      case 0:
        Car_SetAction(CAR_FORWARD);
        break;
      case 1:
        Car_SetAction(CAR_STOP);
        break;
      case 2:
        Car_SetAction(CAR_LEFT);
        break;
      case 3:
        Car_SetAction(CAR_RIGHT);
        break;
      case 4:
        Car_SetAction(CAR_BACKWARD);
        break;
      default:
        Car_SetAction(CAR_STOP);
        break;
    }

    // PID控制 - 每10ms执行一次，与编码器更新同步
    if (HAL_GetTick() - last_pid_tick >= 10)
    {
        Car_PID_Control();
        last_pid_tick = HAL_GetTick();
    }

    // 发送编码器数据（内部已有100ms速率限制）
    SendEncoderData();

    // 发送雷达数据（内部已有100ms速率限制）
    SendRadarData();

    // Small delay to prevent CPU overload, but allow fast odometry updates
    HAL_Delay(1); // 1ms delay instead of 50ms to allow 10ms odometry updates
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 处理蓝牙模块接收到的数据
        Bluetooth_RxCpltCallback(huart);
    }
    else if (huart->Instance == USART3)
    {
        uint8_t Res = uart3_rx_data;
        uint8_t temp;
        
        // 雷达数据接收区 - 使用状态机逐字节接收
        if (uart3_rx_con < 3)
        {
            if(uart3_rx_con == 0)  //接收帧头1 
            {
                //判断帧头1 (0x0A)
                if((Res>>4) == LS_HEADER1)
                {
                    uart3_rx_buf[uart3_rx_con] = Res;
                    uart3_rx_con = 1;                    
                }
            }
            else if(uart3_rx_con == 1) //接收帧头2
            {
                //判断帧头2 (0x05)
                if((Res>>4) == LS_HEADER2)
                {
                    uart3_rx_buf[uart3_rx_con] = Res;
                    uart3_rx_con = 2;
                }
                else
                {
                    uart3_rx_con = 0;                        
                }                
            }
            else  //接收第一个数据
            {
                uart3_rx_buf[uart3_rx_con] = Res;
                uart3_rx_con = 3;
                
                //赋值校验
                uart3_rx_chksum = Res;	
            }
        }			
        else  //接收数据
        {
            //判断是否接收完
            if(uart3_rx_con < (LS_F_LEN-1))
            {
                uart3_rx_buf[uart3_rx_con] = Res;
                uart3_rx_con++;
                uart3_rx_chksum = uart3_rx_chksum^Res;
            }
            else
            {
                //接收最后一个数据
                uart3_rx_buf[uart3_rx_con] = Res;
                uart3_rx_chksum = uart3_rx_chksum^Res;
                
                //复位
                uart3_rx_con = 0;
                
                //计算传输的校验数据
                temp = ((uint8_t)(uart3_rx_buf[1]<<4)) + (uint8_t)(uart3_rx_buf[0]&0x0F);
                
                //判断校验是否正确
                if( uart3_rx_chksum == temp)
                {
                    //接收完毕，进行帧数据处理
                    LS_DataHandle();	
                }
            }
        }
        
        // 重新开启中断
        if (HAL_UART_Receive_IT(&huart3, &uart3_rx_data, 1) != HAL_OK) {
            // 如果重新启动中断失败，可以在这里处理错误
            // 暂时不处理，避免在中断中调用复杂函数
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
