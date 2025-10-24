#include "pid.h"

// PID initialization (set parameters)
void PID_Init(PID *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Integral = 0.0f;
    pid->Err_prev = 0.0f;
}

// PID calculation (positional PID algorithm)
float PID_Calc(PID *pid, float set_speed, float actual_speed)
{
    float err = set_speed - actual_speed; // Calculate error

    // Proportional term
    float P = pid->Kp * err;

    // Integral term (with anti-windup, range: -1000 ~ 1000)
    pid->Integral += err;
    if (pid->Integral > 1000) pid->Integral = 1000;
    if (pid->Integral < -1000) pid->Integral = -1000;
    float I = pid->Ki * pid->Integral;

    // Derivative term
    float D = pid->Kd * (err - pid->Err_prev);
    pid->Err_prev = err;

    // Total output
    float output = P + I + D;

    // Output clamping (PWM range: 0 ~ 100, adjust as needed)
    if (output > 100) output = 100;
    if (output < 0) output = 0;

    // Store output in PID structure for monitoring
    pid->Output = output;

    return output;
} 