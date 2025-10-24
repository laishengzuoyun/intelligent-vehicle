"""
里程计模块 - 基于编码器数据计算小车位置
"""

import math
import config


class Odometry:
    """里程计类 - 计算小车位姿"""
    
    def __init__(self):
        # 小车参数（从config读取）
        self.wheel_diameter = config.WHEEL_DIAMETER  # 轮子直径（mm）
        self.wheel_base = config.WHEEL_BASE  # 轮距（mm）
        self.encoder_resolution = config.ENCODER_RESOLUTION  # 编码器分辨率（脉冲/圈）
        
        # 当前位姿
        self.x = 0.0  # mm
        self.y = 0.0  # mm
        self.theta = 0.0  # 度
        
        # 上一次的编码器读数
        self.last_left_rev = 0.0
        self.last_right_rev = 0.0
        self.first_update = True
        
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """重置位姿"""
        self.x = x
        self.y = y
        self.theta = theta
        self.first_update = True
        
    def update_from_encoder(self, left_rev, right_rev):
        """根据编码器数据更新位姿
        
        参数:
            left_rev: 左轮转过的圈数
            right_rev: 右轮转过的圈数
        """
        # 第一次更新，只记录初始值
        if self.first_update:
            self.last_left_rev = left_rev
            self.last_right_rev = right_rev
            self.first_update = False
            return
        
        # 计算增量
        delta_left_rev = left_rev - self.last_left_rev
        delta_right_rev = right_rev - self.last_right_rev
        
        # 更新上一次的值
        self.last_left_rev = left_rev
        self.last_right_rev = right_rev
        
        # 计算左右轮移动距离（mm）
        wheel_circumference = math.pi * self.wheel_diameter
        left_distance = delta_left_rev * wheel_circumference
        right_distance = delta_right_rev * wheel_circumference
        
        # 计算小车移动距离和转角
        distance = (left_distance + right_distance) / 2.0
        delta_theta_rad = (right_distance - left_distance) / self.wheel_base
        delta_theta_deg = math.degrees(delta_theta_rad)
        
        # 更新位姿（使用中点法）
        theta_mid_rad = math.radians(self.theta + delta_theta_deg / 2.0)
        
        self.x += distance * math.cos(theta_mid_rad)
        self.y += distance * math.sin(theta_mid_rad)
        self.theta += delta_theta_deg
        
        # 归一化角度到[0, 360)
        self.theta = self.theta % 360
        
    def get_pose(self):
        """获取当前位姿"""
        return self.x, self.y, self.theta
    
    def update_from_imu(self, imu_theta):
        """使用IMU数据校正角度
        
        参数:
            imu_theta: IMU测量的角度（度）
        """
        # 可以使用互补滤波或卡尔曼滤波融合编码器和IMU数据
        # 这里简单地使用加权平均
        alpha = 0.7  # 编码器权重
        self.theta = alpha * self.theta + (1 - alpha) * imu_theta
        self.theta = self.theta % 360


class OdometryWithIMU(Odometry):
    """带IMU融合的里程计"""
    
    def __init__(self):
        super().__init__()
        self.imu_available = False
        
    def update(self, left_rev, right_rev, imu_theta=None):
        """更新位姿（可选IMU数据）
        
        参数:
            left_rev: 左轮转过的圈数
            right_rev: 右轮转过的圈数
            imu_theta: IMU测量的角度（度），可选
        """
        # 先用编码器更新
        self.update_from_encoder(left_rev, right_rev)
        
        # 如果有IMU数据，融合角度
        if imu_theta is not None:
            self.update_from_imu(imu_theta)
            self.imu_available = True


class SimpleOdometry:
    """简化版里程计 - 直接使用STM32发送的位姿数据"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
    def update_from_pose(self, x, y, theta):
        """直接更新位姿（STM32已经计算好）"""
        self.x = x
        self.y = y
        self.theta = theta
        
    def get_pose(self):
        """获取当前位姿"""
        return self.x, self.y, self.theta
    
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """重置位姿"""
        self.x = x
        self.y = y
        self.theta = theta


# 使用示例
if __name__ == "__main__":
    # 测试里程计
    odom = Odometry()
    
    print("初始位姿:", odom.get_pose())
    
    # 模拟编码器数据
    # 假设小车直线前进1圈
    odom.update_from_encoder(0.0, 0.0)  # 第一次更新
    odom.update_from_encoder(1.0, 1.0)  # 前进1圈
    
    print("前进1圈后:", odom.get_pose())
    
    # 假设小车原地左转
    odom.update_from_encoder(1.0, 2.0)  # 右轮多转1圈
    
    print("左转后:", odom.get_pose())

