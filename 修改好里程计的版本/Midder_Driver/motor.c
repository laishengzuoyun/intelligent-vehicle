#include "motor.h"
#include "tim.h"
#include "gpio.h"
#include "encoder.h"
extern TIM_HandleTypeDef htim3;

// === PID控制相关全局变量 ===
PID left_pid, right_pid;
float target_left_speed = 0.0f, target_right_speed = 0.0f;
uint8_t pid_enabled = 0;
uint8_t pid_startup_phase = 0; // PID启动阶段标志
uint32_t pid_startup_timer = 0; // PID启动计时器


// === 小车初始化 ===
void Car_Init(void) {
    // 启动 PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // 初始化PID控制器
    Car_PID_Init();

    // 默认停止小车
    Car_SetAction(CAR_STOP);
}

// === PID控制器初始化 ===
void Car_PID_Init(void)
{
    // 初始化左轮PID控制器 (Kp, Ki, Kd)
    // 左轮参数: Kp=3.0, Ki=0.1, Kd=0.05
    PID_Init(&left_pid, 3.0f, 0.1f, 0.05f);

    // 初始化右轮PID控制器 (Kp, Ki, Kd)
    // 右轮参数更大: Kp=4.0, Ki=0.15, Kd=0.08
    // 原因: 右轮电机可能存在机械差异或负载不对称，需要更强的控制增益
    // 这些参数应该通过实际测试调整以达到最佳性能
    PID_Init(&right_pid, 4.0f, 0.15f, 0.08f);

    // 设置初始目标速度
    target_left_speed = 0.0f;
    target_right_speed = 0.0f;
    pid_enabled = 0;
}

// === 设置目标速度 ===
void Car_SetTargetSpeed(float left_speed, float right_speed)
{
    target_left_speed = left_speed;
    target_right_speed = right_speed;
}

// === PID控制函数 ===
void Car_PID_Control(void)
{
    if (!pid_enabled) return;
    
    // PID启动阶段：使用固定PWM值启动
    if (pid_startup_phase) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - pid_startup_timer < 1000) { // 启动阶段持续1秒
            // 使用固定PWM值启动 - 左轮60，右轮62
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 60);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 62);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            return;
        } else {
            // 启动阶段结束，进入PID控制
            pid_startup_phase = 0;
        }
    }
    
    // 计算左轮PID输出
    float left_output = PID_Calc(&left_pid, target_left_speed, left_speed_mps);
    
    // 计算右轮PID输出
    float right_output = PID_Calc(&right_pid, target_right_speed, right_speed_mps);
    
    // 限制输出范围
    if (left_output > 100) left_output = 100;
    if (left_output < 0) left_output = 0;
    if (right_output > 100) right_output = 100;
    if (right_output < 0) right_output = 0;
    
    // 确保最小输出值，防止小车不动
    if (left_output < 20 && target_left_speed > 0) left_output = 20;
    if (right_output < 20 && target_right_speed > 0) right_output = 20;
    
    // 设置PWM输出
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)left_output);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)right_output);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

// === 小车动作设置 ===
void Car_SetAction(CarAction action) {
    switch (action) {
        case CAR_STOP:
            pid_enabled = 0; // 禁用PID控制
            pid_startup_phase = 0; // 重置启动阶段
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;

        case CAR_FORWARD:
            pid_enabled = 1; // 启用PID控制
            pid_startup_phase = 1; // 进入启动阶段
            pid_startup_timer = HAL_GetTick(); // 记录启动时间
            // 设置目标速度 (m/s)
            Car_SetTargetSpeed(0.3f, 0.3f);
            break;

        case CAR_BACKWARD:
            pid_enabled = 0; // 禁用PID控制，使用直接控制
            pid_startup_phase = 0; // 清除启动阶段标志
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 50);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 51);
            break;

        case CAR_LEFT:
            pid_enabled = 0; // 禁用PID控制，使用直接控制
            pid_startup_phase = 0; // 清除启动阶段标志
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 50);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 51);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;

        case CAR_RIGHT:
            pid_enabled = 0; // 禁用PID控制，使用直接控制
            pid_startup_phase = 0; // 清除启动阶段标志
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 51);
            break;

        default:
            break;
    }
}
