#ifndef __ODOMETRY_H
#define __ODOMETRY_H

typedef struct {
    float x;      // 当前位置 X (m)
    float y;      // 当前位置 Y (m)
    float theta;  // 当前朝向 (rad)
} OdometryPose;

void Odometry_Init(void);
void Odometry_Update(float gyro_z_dps, float v_l, float v_r);
OdometryPose Odometry_GetPose(void);

float Odometry_GetX(void);
float Odometry_GetY(void);
float Odometry_GetTheta(void);

#endif
