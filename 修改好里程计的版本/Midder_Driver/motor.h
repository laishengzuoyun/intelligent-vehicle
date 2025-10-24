#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "pid.h"

// === 小车动作枚举 ===
typedef enum {
		CAR_FORWARD = 0,  
    CAR_STOP = 1,
    CAR_LEFT = 2,
    CAR_RIGHT = 3,
	  CAR_BACKWARD = 4,
} CarAction;

// === 小车初始化 ===
void Car_Init(void);

// === 设置小车动作 ===
void Car_SetAction(CarAction action);

// === PID控制相关函数 ===
void Car_PID_Init(void);
void Car_PID_Control(void);
void Car_SetTargetSpeed(float left_speed, float right_speed);

// === 全局变量声明 ===
extern PID left_pid, right_pid;
extern float target_left_speed, target_right_speed;
extern uint8_t pid_enabled;

#endif /* __MOTOR_H */
