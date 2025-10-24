#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"

// === 全局可访问的测速结果 ===
extern float left_speed_mps;
extern float right_speed_mps;
extern float left_total_revs;
extern float right_total_revs;

// === 初始化函数 ===
void Encoder_Init(void);

// === 定时更新采样 ===
void Encoder_Update(void);

#endif
