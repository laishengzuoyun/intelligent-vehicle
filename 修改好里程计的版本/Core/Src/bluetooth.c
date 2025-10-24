#include "bluetooth.h"
#include "radar.h" // 雷达模块头文件
#include "encoder.h" // 编码器模块头文件
#include "motor.h" // 电机控制模块头文件
#include "tim.h" // 定时器头文件
#include "odometry.h"//里程计头文件
#include "mpu6500.h" // MPU6500陀螺仪头文件
#include "i2c.h" // I2C头文件
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef* bluetooth_uart;
static uint8_t rx_data;

#define CMD_BUFFER_SIZE 20
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;

// 编码器数据发送控制
static uint8_t encoder_data_sending = 0;
static uint8_t pid_monitor_enabled = 0;

// 雷达命令标志位
static uint8_t radar_start_requested = 0;
static uint8_t radar_stop_requested = 0;

// 雷达数据发送控制
extern LaserPointTypeDef ax_ls_point[250];  // 雷达数据数组（来自radar.c）
static uint8_t radar_data_sending = 0;  // 雷达数据发送标志

// 外部变量声明
extern uint8_t pid_startup_phase;
extern I2C_HandleTypeDef hi2c1;  // I2C句柄
extern MPU6500_Calib mpu_ctx;    // MPU6500校准上下文

void Bluetooth_Init(UART_HandleTypeDef* huart)
{
    bluetooth_uart = huart;
    HAL_UART_Receive_IT(bluetooth_uart, &rx_data, 1);
}

extern volatile uint8_t car_command;

void ProcessBluetoothCommand(const char* cmd) {
    if (strcmp(cmd, "7") == 0) {
        radar_start_requested = 1;
    } else if (strcmp(cmd, "8") == 0) {
        radar_stop_requested = 1;
    } else if (strcmp(cmd, "9") == 0) {
        AX_LASER_Test();
    } else if (strcmp(cmd, "R") == 0) {
        AX_LASER_GetData();
    } else if (strcmp(cmd, "T") == 0) {
        // 发送EXPRESS_SCAN命令（传统模式）
        // 命令格式: A5 82 05 M 00 00 00 00 22
        // M = 0x00 表示传统模式
        uint8_t std_cmd[] = {0xA5, EXPRESS_SCAN_CMD, EXPRESS_SCAN_LEN, 0x00, 0x00, 0x00, 0x00, 0x00, EXPRESS_SCAN_CHECK};
        HAL_UART_Transmit(&huart3, std_cmd, 9, 1000);
        
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Standard mode radar start sent\r\n", 32, 100);
    }
    // PID参数调节命令
    else if (strncmp(cmd, "PID_L_", 6) == 0) {
        float kp, ki, kd;
        if (sscanf(cmd + 6, "%f,%f,%f", &kp, &ki, &kd) == 3) {
            PID_Init(&left_pid, kp, ki, kd);
            char resp[50];
            sprintf(resp, "Left PID set: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
            HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
        }
    }
    else if (strncmp(cmd, "PID_R_", 6) == 0) {
        float kp, ki, kd;
        if (sscanf(cmd + 6, "%f,%f,%f", &kp, &ki, &kd) == 3) {
            PID_Init(&right_pid, kp, ki, kd);
            char resp[50];
            sprintf(resp, "Right PID set: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
            HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
        }
    }
    else if (strncmp(cmd, "SPEED_", 6) == 0) {
        float left_speed, right_speed;
        if (sscanf(cmd + 6, "%f,%f", &left_speed, &right_speed) == 2) {
            Car_SetTargetSpeed(left_speed, right_speed);
            char resp[50];
            sprintf(resp, "Target speed set: L=%.2f, R=%.2f m/s\r\n", left_speed, right_speed);
            HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
        }
    }
    else if (strcmp(cmd, "PID_DEBUG") == 0) {
        char resp[150];
        sprintf(resp, "PID Status: enabled=%d, startup=%d, target_L=%.2f, target_R=%.2f, actual_L=%.2f, actual_R=%.2f\r\n", 
                pid_enabled, pid_startup_phase, target_left_speed, target_right_speed, left_speed_mps, right_speed_mps);
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "PID_MONITOR") == 0) {
        // 开始PID监控，持续发送PID状态
        pid_monitor_enabled = !pid_monitor_enabled;
        if (pid_monitor_enabled) {
            const char* resp = "PID monitoring started\r\n";
            HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
        } else {
            const char* resp = "PID monitoring stopped\r\n";
            HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
        }
    }
    else if (strcmp(cmd, "SPEED_COMPARE") == 0) {
        // 比较左右轮速度
        char resp[100];
        float speed_diff = left_speed_mps - right_speed_mps;
        sprintf(resp, "Speed Compare: L=%.3f, R=%.3f, Diff=%.3f (L-R)\r\n", 
                left_speed_mps, right_speed_mps, speed_diff);
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "TEST_MOTOR") == 0) {
        // 测试电机，直接设置PWM
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 30);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 30);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        const char* resp = "Motor test started (PWM=30)\r\n";
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "STOP_MOTOR") == 0) {
        // 停止电机
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        const char* resp = "Motor stopped\r\n";
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "ENC_TEST") == 0) {
        // 编码器诊断：显示原始计数器值
        char resp[150];
        int left_cnt = (short)__HAL_TIM_GET_COUNTER(&htim2);
        int right_cnt = (short)__HAL_TIM_GET_COUNTER(&htim4);
        sprintf(resp, "Encoder Test:\r\nLeft CNT=%d, Right CNT=%d\r\nLeft Speed=%.3f m/s, Right Speed=%.3f m/s\r\n",
                left_cnt, right_cnt, left_speed_mps, right_speed_mps);
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "ENC_RESET") == 0) {
        // 重置编码器计数
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        left_total_revs = 0.0f;
        right_total_revs = 0.0f;
        const char* resp = "Encoder counters reset\r\n";
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "POSE_RESET") == 0) {
        // 重置位姿（包括陀螺仪零点）
        Odometry_Init();  // 重置位姿为(0,0,0)
        MPU6500_Calibrate(&hi2c1, &mpu_ctx);  // 校准陀螺仪零点
        const char* resp = "Pose reset and gyro calibrated\r\n";
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else if (strcmp(cmd, "GYRO_CAL") == 0) {
        // 仅校准陀螺仪零点（小车必须静止）
        MPU6500_Calibrate(&hi2c1, &mpu_ctx);
        const char* resp = "Gyro calibration done (keep robot still!)\r\n";
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
    else {
        const char* resp = "Unknown command\r\n";
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)resp, strlen(resp), 100);
    }
}

// 发送编码器数据
void SendEncoderData(void)
{
    static uint32_t last_send_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // 限制发送频率为每100ms一次（10Hz），避免UART阻塞
    if (current_tick - last_send_tick < 100) {
        return;
    }
    last_send_tick = current_tick;

    if (encoder_data_sending)
    {
        char data_buffer[100];
        sprintf(data_buffer, "L:%.2f,R:%.2f,LS:%.2f,RS:%.2f\r\n",
                left_total_revs, right_total_revs, left_speed_mps, right_speed_mps);
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)data_buffer, strlen(data_buffer), 100);
    }

    // PID监控功能
    if (pid_monitor_enabled && pid_enabled)
    {
        char data_buffer[150];
        sprintf(data_buffer, "PID: L_out=%.1f,R_out=%.1f,target_L=%.2f,target_R=%.2f,actual_L=%.2f,actual_R=%.2f\r\n",
                left_pid.Output, right_pid.Output, target_left_speed, target_right_speed, left_speed_mps, right_speed_mps);
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)data_buffer, strlen(data_buffer), 100);
    }
}

// 发送雷达数据到PC
void SendRadarData(void)
{
    static uint32_t last_send_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // 优化：发送频率提高到每100ms一次（10Hz），提高实时性
    if (current_tick - last_send_tick < 100) {
        return;
    }
    last_send_tick = current_tick;

    if (!radar_data_sending) {
        return;
    }

    // 优化：每次发送10个点，加快扫描速度（完整扫描从10秒降到2.5秒）
    static uint16_t send_index = 0;
    uint16_t sent_count = 0;

    for (uint16_t i = 0; i < 250 && sent_count < 10; i++)
    {
        // 只发送有效数据（距离>0且距离<6000mm）
        if (ax_ls_point[send_index].distance > 0 && ax_ls_point[send_index].distance < 6000) {
            char data_buffer[64];
            float angle = ax_ls_point[send_index].angle / 100.0f;  // 转换为度
            uint16_t distance = ax_ls_point[send_index].distance;  // mm

            sprintf(data_buffer, "Point %d: A=%.1f° D=%umm Q=0\r\n",
                    send_index, angle, distance);

            // 增加超时时间到100ms，确保数据发送完整
            if (HAL_UART_Transmit(bluetooth_uart, (uint8_t*)data_buffer, strlen(data_buffer), 100) == HAL_OK) {
                sent_count++;
                // 优化：减少延迟到3ms，加快发送速度
                HAL_Delay(3);
            }
        }

        send_index++;
        if (send_index >= 250) {
            send_index = 0;
        }
    }
}

void Bluetooth_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == bluetooth_uart)
    {
        if (rx_data == '\r' || rx_data == '\n') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0'; // 字符串结束符
                
                // 调试：显示接收到的命令
                char debug_msg[50];
                sprintf(debug_msg, "Cmd: '%s'\r\n", cmd_buffer);
                HAL_UART_Transmit(bluetooth_uart, (uint8_t*)debug_msg, strlen(debug_msg), 10);
                
                ProcessBluetoothCommand(cmd_buffer);
                cmd_index = 0;
            }
        }
        else if (cmd_index < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = rx_data;
        }
        else {
            // 超出缓冲区，重置索引
            cmd_index = 0;
        }

        // 处理数字指令控制小车（只在不是命令字符时处理）
        if (rx_data != '\r' && rx_data != '\n') {
            switch (rx_data)
            {
                case '0':
                    car_command = 0;
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Car: FORWARD\r\n", 14, 100);
                    break;
                case '1':
                    car_command = 1;
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Car: STOP\r\n", 11, 100);
                    break;
                case '2':
                    car_command = 2;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Car: TURN LEFT (rotate)\r\n", 25, 100);
                    break;
                case '3':
                    car_command = 3;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Car: TURN RIGHT (rotate)\r\n", 26, 100);
                    break;
                case '4':
                    car_command = 4;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Car: BACKWARD\r\n", 15, 100);
                    break;
                case '5':
                    encoder_data_sending = 1;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Encoder data sending started\r\n", 30, 100);
                    break;
                case '6':
                    encoder_data_sending = 0;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Encoder data sending stopped\r\n", 30, 100);
                    break;
                case '7':
                    radar_data_sending = 1;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Radar data sending started\r\n", 28, 100);
                    break;
                case '8':
                    radar_data_sending = 0;
                    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Radar data sending stopped\r\n", 28, 100);
                    break;
                default:
                    break; // 其他字符不处理
            }
        }

        HAL_UART_Receive_IT(bluetooth_uart, &rx_data, 1);
    }
}

void Bluetooth_SendPose(void)
{
    OdometryPose pose = Odometry_GetPose();
    char msg[128];

    // 添加速度信息用于调试（前10次发送）
    static uint8_t debug_count = 0;
    if (debug_count < 10) {
        snprintf(msg, sizeof(msg), "X:%.2f Y:%.2f T:%.2f° [V_L=%.3f V_R=%.3f]\r\n",
                 pose.x, pose.y, pose.theta * 57.3f, left_speed_mps, right_speed_mps);
        debug_count++;
    } else {
        snprintf(msg, sizeof(msg), "X:%.2f Y:%.2f T:%.2f°\r\n",
                 pose.x, pose.y, pose.theta * 57.3f);
    }

    HAL_UART_Transmit(bluetooth_uart, (uint8_t*)msg, strlen(msg), 100);
}

// 处理雷达命令（在主循环中调用，不在中断中）
void Bluetooth_ProcessRadarCommands(void)
{
    if (radar_start_requested) {
        radar_start_requested = 0;
        AX_LASER_Start();
        radar_data_sending = 1;  // 启动雷达数据发送！
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Radar scanning started\r\n", 24, 100);
    }

    if (radar_stop_requested) {
        radar_stop_requested = 0;
        AX_LASER_Stop();
        radar_data_sending = 0;  // 停止雷达数据发送
        HAL_UART_Transmit(bluetooth_uart, (uint8_t*)"Radar scanning stopped\r\n", 24, 100);
    }
}
