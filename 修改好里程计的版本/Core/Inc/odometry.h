#ifndef __ODOMETRY_H
#define __ODOMETRY_H

typedef struct {
    float x;      // ��ǰλ�� X (m)
    float y;      // ��ǰλ�� Y (m)
    float theta;  // ��ǰ���� (rad)
} OdometryPose;

void Odometry_Init(void);
void Odometry_Update(float gyro_z_dps, float v_l, float v_r);
OdometryPose Odometry_GetPose(void);

float Odometry_GetX(void);
float Odometry_GetY(void);
float Odometry_GetTheta(void);

#endif
