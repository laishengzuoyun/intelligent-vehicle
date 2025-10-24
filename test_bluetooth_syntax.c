/*
 * 语法检查测试
 * 检查bluetooth.c文件的基本语法结构
 */

// 模拟必要的头文件
typedef struct {
    float x, y, theta;
} OdometryPose;

typedef struct {
    float Output;
} PID_TypeDef;

// 模拟必要的变量和函数
extern uint8_t left_speed_mps, right_speed_mps;
extern PID_TypeDef left_pid, right_pid;
extern float target_left_speed, target_right_speed;
extern uint8_t pid_enabled;

// 模拟HAL函数
void HAL_UART_Transmit(void* uart, void* data, int len, int timeout) {}

// 模拟其他函数
OdometryPose Odometry_GetPose(void) {
    OdometryPose pose = {0, 0, 0};
    return pose;
}

// 测试Bluetooth_SendPose函数
void Bluetooth_SendPose(void)
{
    OdometryPose pose = Odometry_GetPose();
    char msg[128];

    // 添加速度信息用于调试（前10次发送）
    static uint8_t debug_count = 0;
    if (debug_count < 10) {
        snprintf(msg, sizeof(msg), "X:%.2f Y:%.2f T:%.2f° [V_L=%.3f V_R=%.3f]\r\n",
                 pose.x, pose.y, pose.theta * 57.3f, left_speed_mps, right_speed_mps);
        debug_count++;
    } else {
        snprintf(msg, sizeof(msg), "X:%.2f Y:%.2f T:%.2f°\r\n",
                 pose.x, pose.y, pose.theta * 57.3f);
    }

    // 高波特率优化：减少超时时间到50ms
    HAL_UART_Transmit(0, (void*)msg, 0, 50);
}







