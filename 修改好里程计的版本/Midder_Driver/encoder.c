#include "encoder.h"
#include "tim.h"
#include <math.h>

/* === 编码器参数 === */
// 每转脉冲数（包括4倍频）
// 若实际轮速不动，请检查是否需调为 6240 / 46800 等
#define ENCODER_PPR 1560.0f  

// 轮周长（单位 m）
#define WHEEL_CIRCUMFERENCE 0.2105f  

// 编码器状态
float left_speed_mps = 0.0f;
float right_speed_mps = 0.0f;
float left_total_revs = 0.0f;
float right_total_revs = 0.0f;

/* === 内部函数 === */
static int Read_Speed(TIM_HandleTypeDef *htim)
{
    int temp = (short)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return temp;
}

/* === 初始化 === */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    left_speed_mps = 0.0f;
    right_speed_mps = 0.0f;
    left_total_revs = 0.0f;
    right_total_revs = 0.0f;
}

/* === 每10ms调用一次（定时器中断）=== */
void Encoder_Update(void)
{
    const float delta_t_s = 0.01f; // 10ms = 0.01s

    // 若左右轮方向相反，可交换符号
    int left_pulse = Read_Speed(&htim2);
    int right_pulse = Read_Speed(&htim4);

    // 转动圈数（保留方向信息）
    float left_delta_revs = (float)left_pulse / ENCODER_PPR;
    float right_delta_revs = (float)right_pulse / ENCODER_PPR;

    // 累计圈数
    left_total_revs += left_delta_revs;
    right_total_revs += right_delta_revs;

    // 线速度（m/s）
    left_speed_mps = (left_delta_revs * WHEEL_CIRCUMFERENCE) / delta_t_s;
    right_speed_mps = (right_delta_revs * WHEEL_CIRCUMFERENCE) / delta_t_s;
}

/* === 提供读取接口 === */
float Encoder_GetLeftSpeed(void)  { return left_speed_mps; }
float Encoder_GetRightSpeed(void) { return right_speed_mps; }

float Encoder_GetLeftDistance(void)
{
    return left_total_revs * WHEEL_CIRCUMFERENCE;
}

float Encoder_GetRightDistance(void)
{
    return right_total_revs * WHEEL_CIRCUMFERENCE;
}
