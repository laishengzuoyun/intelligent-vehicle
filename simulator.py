"""
虚拟小车模拟器 - 无需硬件即可测试所有功能
模拟蓝牙通信、雷达扫描、小车运动
"""

import numpy as np
import time
import math
from PyQt5.QtCore import QThread, pyqtSignal
import config


class VirtualRobotSimulator(QThread):
    """虚拟机器人模拟器"""
    
    # 信号定义
    pose_updated = pyqtSignal(float, float, float)  # x, y, theta
    encoder_data_updated = pyqtSignal(float, float, float, float)  # left_rev, right_rev, left_speed, right_speed
    lidar_data_updated = pyqtSignal(list)  # [(angle, distance), ...]
    data_received = pyqtSignal(str)  # 原始数据
    
    def __init__(self):
        super().__init__()
        
        # 机器人状态 - 使用配置文件中的初始位置
        self.x = config.ROBOT_INIT_X  # mm - 使用配置的初始位置
        self.y = config.ROBOT_INIT_Y  # mm - 使用配置的初始位置
        self.theta = config.ROBOT_INIT_THETA  # degrees - 使用配置的初始朝向
        
        # 运动状态
        self.command = 1  # 0=前进, 1=停止, 2=左转, 3=右转, 4=后退
        self.linear_speed = 50.0  # mm/s - 对应STM32的0.05m/s (0.05m/s = 50mm/s)
        self.angular_speed = 45.0  # deg/s
        
        # 编码器状态
        self.left_revs = 0.0
        self.right_revs = 0.0
        self.left_speed = 0.0  # m/s
        self.right_speed = 0.0  # m/s
        
        # 数据发送标志
        self.encoder_sending = False
        self.radar_sending = False
        self.running = False
        
        # 虚拟地图（用于雷达模拟）
        self.virtual_map = None
        self.load_virtual_map()
        
        # 时间控制
        self.last_update_time = time.time()
        
    def load_virtual_map(self):
        """加载虚拟地图（4×4方格迷宫，280cm×280cm）"""
        # 创建一个4×4方格的迷宫地图
        width = config.MAP_WIDTH  # 2800mm
        height = config.MAP_HEIGHT  # 2800mm

        # 0=空闲, 1=障碍物
        self.virtual_map = np.zeros((height // 10, width // 10), dtype=np.uint8)

        # 添加外墙
        self.virtual_map[0, :] = 1
        self.virtual_map[-1, :] = 1
        self.virtual_map[:, 0] = 1
        self.virtual_map[:, -1] = 1

        # 添加内部墙壁（模拟4×4迷宫）
        # 每个单元格70cm = 700mm = 70个栅格（10mm/格）

        # 垂直墙（示例）
        self.virtual_map[70:140, 70] = 1   # 第1列和第2列之间
        self.virtual_map[140:210, 140] = 1  # 第2列和第3列之间

        # 水平墙（示例）
        self.virtual_map[70, 0:70] = 1     # 第1行和第2行之间
        self.virtual_map[140, 140:210] = 1  # 第2行和第3行之间
        
    def send_command(self, cmd):
        """接收控制命令"""
        if cmd == '0':
            self.command = 0
            self.data_received.emit("Car: FORWARD")
        elif cmd == '1':
            self.command = 1
            self.data_received.emit("Car: STOP")
        elif cmd == '2':
            self.command = 2
            self.data_received.emit("Car: TURN LEFT (rotate)")
        elif cmd == '3':
            self.command = 3
            self.data_received.emit("Car: TURN RIGHT (rotate)")
        elif cmd == '4':
            self.command = 4
            self.data_received.emit("Car: BACKWARD")
        elif cmd == '5':
            self.encoder_sending = True
            self.data_received.emit("Encoder data sending started")
        elif cmd == '6':
            self.encoder_sending = False
            self.data_received.emit("Encoder data sending stopped")
        elif cmd == '7':
            self.radar_sending = True
            self.data_received.emit("Radar data sending started")
        elif cmd == '8':
            self.radar_sending = False
            self.data_received.emit("Radar data sending stopped")
            
    def update_robot_state(self, dt):
        """更新机器人状态"""
        if self.command == 0:  # 前进
            dx = self.linear_speed * dt * math.cos(math.radians(self.theta))
            dy = self.linear_speed * dt * math.sin(math.radians(self.theta))
            self.x += dx
            self.y += dy
            self.left_speed = 0.05  # 对应STM32的0.05m/s
            self.right_speed = 0.05  # 对应STM32的0.05m/s
            # 删除频繁的DEBUG输出，提升性能
            
        elif self.command == 2:  # 左转
            self.theta += self.angular_speed * dt
            if self.theta > 360:
                self.theta -= 360
            self.left_speed = -0.1
            self.right_speed = 0.1
            
        elif self.command == 3:  # 右转
            self.theta -= self.angular_speed * dt
            if self.theta < 0:
                self.theta += 360
            self.left_speed = 0.1
            self.right_speed = -0.1
            
        elif self.command == 4:  # 后退
            dx = -self.linear_speed * dt * math.cos(math.radians(self.theta))
            dy = -self.linear_speed * dt * math.sin(math.radians(self.theta))
            self.x += dx
            self.y += dy
            self.left_speed = -0.05  # 对应STM32的0.05m/s
            self.right_speed = -0.05  # 对应STM32的0.05m/s
            # 删除频繁的DEBUG输出，提升性能
            
        else:  # 停止
            self.left_speed = 0.0
            self.right_speed = 0.0
            
        # 更新编码器
        self.left_revs += abs(self.left_speed) * dt * 10
        self.right_revs += abs(self.right_speed) * dt * 10
        
    def simulate_lidar_scan(self):
        """模拟雷达扫描 - 优化版本"""
        lidar_data = []
        
        # 优化：减少扫描点数，从每2度改为每5度，减少计算量
        for angle_deg in range(0, 360, 5):  # 每5度一个点，共72个点（减少60%计算量）
            # 计算全局角度
            global_angle = self.theta + angle_deg
            if global_angle > 360:
                global_angle -= 360
                
            # 射线追踪
            distance = self.raycast(global_angle)
            
            if distance > 0:
                lidar_data.append((angle_deg, distance))
                
        return lidar_data
        
    def raycast(self, angle_deg):
        """射线追踪，返回距离（mm） - 优化版本"""
        max_range = 5000  # 5米
        step = 20  # 优化：从10mm增加到20mm步长，减少50%计算量
        
        angle_rad = math.radians(angle_deg)
        
        for dist in range(0, max_range, step):
            # 计算射线上的点
            x = self.x + dist * math.cos(angle_rad)
            y = self.y + dist * math.sin(angle_rad)
            
            # 转换为地图坐标
            map_x = int(x / 10)
            map_y = int(y / 10)
            
            # 检查是否越界
            if map_x < 0 or map_x >= self.virtual_map.shape[1]:
                return dist
            if map_y < 0 or map_y >= self.virtual_map.shape[0]:
                return dist
                
            # 检查是否碰到障碍物
            if self.virtual_map[map_y, map_x] == 1:
                return dist
                
        return max_range
        
    def run(self):
        """主循环"""
        self.running = True
        
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            # 更新机器人状态
            self.update_robot_state(dt)
            
            # 发送位姿数据（1Hz）
            self.pose_updated.emit(self.x, self.y, self.theta)
            
            # 发送编码器数据（10Hz）
            if self.encoder_sending:
                self.encoder_data_updated.emit(
                    self.left_revs, self.right_revs,
                    self.left_speed, self.right_speed
                )
                
            # 发送雷达数据（降低频率：从5Hz降到2Hz）
            if self.radar_sending:
                lidar_data = self.simulate_lidar_scan()
                self.lidar_data_updated.emit(lidar_data)
                
            # 控制更新频率
            time.sleep(0.1)  # 10Hz
            
    def stop(self):
        """停止模拟器"""
        self.running = False
        self.wait()

