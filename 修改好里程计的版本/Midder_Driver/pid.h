// pid.h
#ifndef PID_H
#define PID_H

typedef struct {
    float SetSpeed;      // Target speed
    float ActualSpeed;   // Actual speed
    float Kp, Ki, Kd;    // PID parameters
    float Err, Err_prev; // Current and previous error
    float Integral;      // Integral term
    float Output;        // PWM output value
} PID;

// Function declarations
void PID_Init(PID *pid, float kp, float ki, float kd);
float PID_Calc(PID *pid, float set_speed, float actual_speed);

#endif