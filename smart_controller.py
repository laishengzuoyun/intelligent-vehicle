"""
智能避障控制器 - 基于示例代码的lookahead + repulsion混合控制
结合路径跟踪和障碍物排斥力，实现更智能的避障
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class Pose:
    """机器人位姿"""
    x: float  # mm
    y: float  # mm
    theta: float  # rad

@dataclass
class PoseSetpoint:
    """目标位姿"""
    x: float  # mm
    y: float  # mm
    theta: float  # rad

@dataclass
class LidarScan:
    """雷达扫描数据"""
    angles: List[float]  # rad
    ranges: List[float]  # mm

@dataclass
class ControllerConfig:
    """控制器配置"""
    lookahead_mm: float = 300.0  # 前瞻距离 (mm)
    k_ang: float = 2.0  # 角度控制增益
    k_avoid: float = 1.5  # 避障增益
    avoid_radius_mm: float = 350.0  # 避障半径 (mm)
    v_max_mmps: float = 50.0  # 最大速度 (mm/s)
    repulse_gain: float = 1.0  # 排斥力增益
    min_obstacle_distance_mm: float = 400.0  # 最小障碍物距离 (mm) - 调整为40cm

class SmartObstacleController:
    """智能避障控制器"""
    
    def __init__(self, config: ControllerConfig):
        self.config = config
        self.path: List[Tuple[float, float]] = []  # 全局路径 (mm)
        self.current_target_index = 0
        
    def set_path(self, path: List[Tuple[float, float]]):
        """设置全局路径"""
        self.path = [(x * 1000, y * 1000) for x, y in path]  # 转换为mm
        self.current_target_index = 0
        
    def _wrap_angle(self, angle: float) -> float:
        """角度归一化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
        
    def _lookahead_target(self, pose: Pose) -> Tuple[float, float]:
        """计算前瞻目标点"""
        if not self.path:
            return (pose.x, pose.y)
            
        # 找到距离当前位置最近的路径点
        dists = [math.hypot(px - pose.x, py - pose.y) for px, py in self.path]
        closest_idx = int(np.argmin(dists))
        
        # 从最近点开始，沿着路径累计距离直到达到前瞻距离
        L = 0.0
        i = closest_idx
        
        while L < self.config.lookahead_mm and i < len(self.path) - 1:
            dx = self.path[i + 1][0] - self.path[i][0]
            dy = self.path[i + 1][1] - self.path[i][1]
            L += math.hypot(dx, dy)
            i += 1
            
        return self.path[i]
        
    def _repulsive_heading(self, scan: LidarScan) -> Tuple[float, float]:
        """计算排斥力方向"""
        R = self.config.avoid_radius_mm
        g = self.config.repulse_gain
        rx = ry = 0.0
        
        for angle_rel, distance in zip(scan.angles, scan.ranges):
            if distance <= 0.0 or distance > R:
                continue
                
            # 计算排斥力强度（距离越近，排斥力越强）
            s = g * max(0.0, (1.0 / distance - 1.0 / R))
            
            # 排斥力方向（远离障碍物）
            rx += -math.cos(angle_rel) * s
            ry += -math.sin(angle_rel) * s
            
        return (rx, ry)
        
    def _emergency_stop_check(self, scan: LidarScan, pose: Pose) -> bool:
        """紧急停车检查 - 300mm安全距离"""
        min_dist = self.config.min_obstacle_distance_mm  # 300mm
        
        # 检查前方±30度范围内的障碍物
        for angle_rel, distance in zip(scan.angles, scan.ranges):
            if abs(angle_rel) <= math.pi / 6:  # ±30度
                if distance < min_dist:  # 300mm安全距离
                    return True
        return False
        
    def compute_setpoint(self, pose: Pose, scan: LidarScan) -> Optional[PoseSetpoint]:
        """计算目标位姿"""
        # 紧急停车检查
        if self._emergency_stop_check(scan, pose):
            return None  # 返回None表示需要停车
            
        # 计算前瞻目标
        gx, gy = self._lookahead_target(pose)
        
        # 计算朝向目标的向量（机器人坐标系）
        ax, ay = gx - pose.x, gy - pose.y
        ct, st = math.cos(pose.theta), math.sin(pose.theta)
        ahead_x_robot = ct * ax + st * ay
        ahead_y_robot = -st * ax + ct * ay
        
        # 计算排斥力（机器人坐标系）
        rx, ry = self._repulsive_heading(scan)
        
        # 组合控制向量
        comb_x = ahead_x_robot + self.config.k_avoid * rx
        comb_y = ahead_y_robot + self.config.k_avoid * ry
        
        # 计算期望朝向
        desired_heading_robot = math.atan2(comb_y, comb_x)
        desired_theta_world = (pose.theta + desired_heading_robot) % (2 * math.pi)
        
        return PoseSetpoint(x=gx, y=gy, theta=desired_theta_world)
        
    def get_debug_info(self, pose: Pose, scan: LidarScan) -> dict:
        """获取调试信息"""
        gx, gy = self._lookahead_target(pose)
        rx, ry = self._repulsive_heading(scan)
        
        # 计算前方障碍物统计
        front_obstacles = []
        for angle_rel, distance in zip(scan.angles, scan.ranges):
            if abs(angle_rel) <= math.pi / 4:  # ±45度
                if distance < self.config.avoid_radius_mm:
                    front_obstacles.append((angle_rel, distance))
                    
        min_distance = min([d for _, d in front_obstacles], default=float('inf'))
        
        return {
            'lookahead_target': (gx, gy),
            'repulsion_force': (rx, ry),
            'front_obstacles_count': len(front_obstacles),
            'min_distance': min_distance,
            'emergency_stop': self._emergency_stop_check(scan, pose)
        }

# 简化的手动控制器（用于测试）
class SimpleManualController:
    """简单手动控制器"""
    
    def __init__(self, config: ControllerConfig):
        self.config = config
        
    def compute_setpoint(self, pose: Pose, scan: LidarScan, direction: str) -> Optional[PoseSetpoint]:
        """根据方向计算目标位姿"""
        step_mm = 50.0  # 每步50mm
        turn_angle = math.pi / 18  # 10度
        
        # 检查前方是否有障碍物
        front_clear = True
        for angle_rel, distance in zip(scan.angles, scan.ranges):
            if abs(angle_rel) <= math.pi / 6:  # ±30度
                if distance < self.config.min_obstacle_distance_mm:  # 300mm安全距离
                    front_clear = False
                    break
                    
        if not front_clear and direction in ['forward', 'backward']:
            return None  # 需要停车
            
        if direction == 'forward' and front_clear:
            new_x = pose.x + step_mm * math.cos(pose.theta)
            new_y = pose.y + step_mm * math.sin(pose.theta)
            return PoseSetpoint(x=new_x, y=new_y, theta=pose.theta)
        elif direction == 'backward':
            new_x = pose.x - step_mm * math.cos(pose.theta)
            new_y = pose.y - step_mm * math.sin(pose.theta)
            return PoseSetpoint(x=new_x, y=new_y, theta=pose.theta)
        elif direction == 'left':
            new_theta = (pose.theta + turn_angle) % (2 * math.pi)
            return PoseSetpoint(x=pose.x, y=pose.y, theta=new_theta)
        elif direction == 'right':
            new_theta = (pose.theta - turn_angle) % (2 * math.pi)
            return PoseSetpoint(x=pose.x, y=pose.y, theta=new_theta)
            
        return None


