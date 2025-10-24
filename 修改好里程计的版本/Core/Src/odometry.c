#include "odometry.h"
#include <math.h>

#define DT 0.01f
#define WHEEL_BASE 0.15f
#define M_PI 3.14159265358979323846f

static OdometryPose pose = {0};

/* ??? */
void Odometry_Init(void)
{
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.theta = 0.0f;
}

/* ?10ms????,??????????????? */
void Odometry_Update(float gyro_z_dps, float v_l, float v_r)
{
    // 1. ?????(?IMU??,?/s -> rad/s)
    float omega_imu = gyro_z_dps * (M_PI / 180.0f);
    float omega_enc = (v_r - v_l) / WHEEL_BASE;

    // ????(????????)
    float omega = 0.7f * omega_imu + 0.3f * omega_enc;

    // 2. ?????
    pose.theta += omega * DT;
    if (pose.theta > M_PI) pose.theta -= 2.0f * M_PI;
    else if (pose.theta < -M_PI) pose.theta += 2.0f * M_PI;

    // 3. ????
    float v = (v_l + v_r) / 2.0f;
    pose.x += v * cosf(pose.theta) * DT;
    pose.y += v * sinf(pose.theta) * DT;
}

/* ?????? */
OdometryPose Odometry_GetPose(void)
{
    return pose;
}
