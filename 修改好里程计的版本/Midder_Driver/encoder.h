#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"

// === ȫ�ֿɷ��ʵĲ��ٽ�� ===
extern float left_speed_mps;
extern float right_speed_mps;
extern float left_total_revs;
extern float right_total_revs;

// === ��ʼ������ ===
void Encoder_Init(void);

// === ��ʱ���²��� ===
void Encoder_Update(void);

#endif
