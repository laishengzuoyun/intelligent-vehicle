"""
地图管理模块
负责栅格地图的创建、更新和管理
"""

import numpy as np
import json
import math
import config


class OccupancyGridMap:
    """占据栅格地图类"""
    
    def __init__(self, width, height, cell_size):
        self.map_width = width
        self.map_height = height
        self.cell_size = cell_size
        self.grid_col_count = int(width / cell_size)
        self.grid_row_count = int(height / cell_size)
        
        # 初始化地图为未知状态
        self.grid_map = np.full(
            (self.grid_row_count, self.grid_col_count),
            config.MAP_STATE_UNKNOWN,
            dtype=np.int8
        )
        
        # 存储墙壁信息（从JSON加载）
        self.walls = []
        
    def world_to_grid(self, x, y):
        """世界坐标转栅格坐标"""
        grid_x = int(x / self.cell_size)
        grid_y = int(y / self.cell_size)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """栅格坐标转世界坐标（中心点）"""
        x = (grid_x + 0.5) * self.cell_size
        y = (grid_y + 0.5) * self.cell_size
        return x, y
    
    def is_valid_grid(self, grid_x, grid_y):
        """检查栅格坐标是否有效"""
        return 0 <= grid_x < self.grid_col_count and 0 <= grid_y < self.grid_row_count

    def initialize_robot_area(self, robot_x, robot_y, radius=200):
        """初始化机器人周围的区域为空闲

        参数:
            robot_x: 机器人X坐标（mm）
            robot_y: 机器人Y坐标（mm）
            radius: 初始化半径（mm），默认200mm
        """
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        radius_cells = int(radius / self.cell_size)

        # 将机器人周围的圆形区域标记为空闲
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                # 检查是否在圆形范围内
                if dx*dx + dy*dy <= radius_cells * radius_cells:
                    gx, gy = robot_gx + dx, robot_gy + dy
                    if self.is_valid_grid(gx, gy):
                        self.grid_map[gy, gx] = config.MAP_STATE_FREE

    def update_from_lidar(self, robot_x, robot_y, robot_theta, lidar_data):
        """根据雷达数据更新地图 - 优化版本

        Args:
            robot_x, robot_y: 机器人世界坐标
            robot_theta: 机器人朝向（度）
            lidar_data: 雷达数据列表 [(angle, distance), ...]
        """
        import time
        
        # 性能优化：限制雷达数据处理频率
        current_time = time.time() * 1000
        if not hasattr(self, '_last_lidar_update_time'):
            self._last_lidar_update_time = 0
        
        # 雷达数据处理间隔：每100ms处理一次，避免过于频繁
        if current_time - self._last_lidar_update_time < config.LIDAR_PROCESSING_INTERVAL:
            return
        
        self._last_lidar_update_time = current_time
        
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)

        # 性能优化：限制处理的雷达点数量
        max_points = config.MAX_LIDAR_POINTS_PER_UPDATE
        if len(lidar_data) > max_points:
            # 均匀采样
            step = len(lidar_data) // max_points
            processed_data = lidar_data[::step]
        else:
            processed_data = lidar_data

        # 批量收集需要更新的栅格点
        free_points = []
        occupied_points = []
        
        for angle, distance in processed_data:
            # 计算雷达点的世界坐标
            # angle是相对于机器人坐标系的角度
            # 雷达数据：0°=前方，90°=右方（顺时针）
            # 机器人位姿：0°=前方，90°=左方（逆时针）
            # 雷达和机器人坐标系相反，需要取反角度
            absolute_angle = robot_theta - angle
            rad = math.radians(absolute_angle)
            
            # 雷达点世界坐标
            point_x = robot_x + distance * math.cos(rad)
            point_y = robot_y + distance * math.sin(rad)
            
            point_grid_x, point_grid_y = self.world_to_grid(point_x, point_y)
            
            # 使用优化的射线算法
            line_points = self._optimized_bresenham_line(
                robot_grid_x, robot_grid_y,
                point_grid_x, point_grid_y
            )
            
            # 收集需要更新的点
            for i, (gx, gy) in enumerate(line_points):
                if self.is_valid_grid(gx, gy):
                    if i == len(line_points) - 1:
                        # 最后一个点：如果距离小于最大量程，标记为障碍物
                        if distance < config.LIDAR_MAX_RANGE * 0.95:
                            occupied_points.append((gx, gy))
                    else:
                        # 路径上的点标记为空闲
                        if self.grid_map[gy, gx] == config.MAP_STATE_UNKNOWN:
                            free_points.append((gx, gy))
        
        # 批量更新栅格地图
        for gx, gy in free_points:
            self.grid_map[gy, gx] = config.MAP_STATE_FREE
        
        for gx, gy in occupied_points:
            self.grid_map[gy, gx] = config.MAP_STATE_OCCUPIED
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham直线算法"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        
        points.append((x, y))
        return points
    
    def _optimized_bresenham_line(self, x0, y0, x1, y1):
        """优化的Bresenham直线算法 - 减少内存分配和计算"""
        # 计算距离，如果太远则跳过
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        # 如果射线太长，使用步长跳跃
        max_length = config.MAX_RAY_LENGTH
        if dx > max_length or dy > max_length:
            # 对于长射线，使用大步长
            step = max(dx, dy) // max_length
            if step < 1:
                step = 1
            
            # 简化的射线算法
            points = []
            x, y = x0, y0
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            
            if dx > dy:
                err = dx / 2.0
                while x != x1 and len(points) < max_length:
                    points.append((x, y))
                    err -= dy
                    if err < 0:
                        y += sy
                        err += dx
                    x += sx * step
            else:
                err = dy / 2.0
                while y != y1 and len(points) < max_length:
                    points.append((x, y))
                    err -= dx
                    if err < 0:
                        x += sx
                        err += dy
                    y += sy * step
            
            points.append((x, y))
            return points
        else:
            # 短射线使用原始算法
            return self._bresenham_line(x0, y0, x1, y1)
    
    def load_from_json(self, json_path):
        """从JSON文件加载地图"""
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            self.walls = []
            
            # 解析墙壁段
            for segment in data['segments']:
                start = segment['start']
                end = segment['end']
                
                # 转换为世界坐标
                x1 = start[0] * config.SCALE_FACTOR
                y1 = start[1] * config.SCALE_FACTOR
                x2 = end[0] * config.SCALE_FACTOR
                y2 = end[1] * config.SCALE_FACTOR
                
                self.walls.append(((x1, y1), (x2, y2)))
                
                # 在地图上标记墙壁
                self._mark_wall_on_grid(x1, y1, x2, y2)
            
            # 获取起点
            start_point = data.get('start_point', [1, 1])
            start_x = start_point[0] * config.SCALE_FACTOR
            start_y = start_point[1] * config.SCALE_FACTOR
            
            return (start_x, start_y), start_point
            
        except Exception as e:
            print(f"加载地图失败: {e}")
            return None, None
    
    def _mark_wall_on_grid(self, x1, y1, x2, y2, thickness=10):
        """在栅格地图上标记墙壁"""
        # 转换为栅格坐标
        gx1, gy1 = self.world_to_grid(x1, y1)
        gx2, gy2 = self.world_to_grid(x2, y2)
        
        # 获取墙壁路径
        wall_points = self._bresenham_line(gx1, gy1, gx2, gy2)
        
        # 考虑墙壁厚度
        thickness_cells = max(1, int(thickness / self.cell_size))
        
        for gx, gy in wall_points:
            for dx in range(-thickness_cells, thickness_cells + 1):
                for dy in range(-thickness_cells, thickness_cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if self.is_valid_grid(nx, ny):
                        self.grid_map[ny, nx] = config.MAP_STATE_OCCUPIED
    
    def get_frontiers(self):
        """获取前沿点（已知空闲区域与未知区域的边界）"""
        frontiers = []
        
        for y in range(self.grid_row_count):
            for x in range(self.grid_col_count):
                if self.grid_map[y, x] == config.MAP_STATE_FREE:
                    # 检查四周是否有未知区域
                    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                        nx, ny = x + dx, y + dy
                        if self.is_valid_grid(nx, ny):
                            if self.grid_map[ny, nx] == config.MAP_STATE_UNKNOWN:
                                frontiers.append((x, y))
                                break
        
        return frontiers
    
    def expand_obstacles(self, expansion_radius):
        """膨胀障碍物（用于路径规划）"""
        expanded_map = np.copy(self.grid_map)
        
        # 找到所有障碍物
        obstacles = np.argwhere(self.grid_map == config.MAP_STATE_OCCUPIED)
        
        for y, x in obstacles:
            for dy in range(-expansion_radius, expansion_radius + 1):
                for dx in range(-expansion_radius, expansion_radius + 1):
                    nx, ny = x + dx, y + dy
                    if self.is_valid_grid(nx, ny):
                        if expanded_map[ny, nx] != config.MAP_STATE_OCCUPIED:
                            expanded_map[ny, nx] = config.MAP_STATE_OCCUPIED
        
        return expanded_map
    
    def get_exploration_progress(self):
        """获取探索进度（百分比）"""
        total_cells = self.grid_row_count * self.grid_col_count
        unknown_cells = np.sum(self.grid_map == config.MAP_STATE_UNKNOWN)
        explored_ratio = (total_cells - unknown_cells) / total_cells
        return explored_ratio * 100


# 为了兼容性，提供MapManager别名
MapManager = OccupancyGridMap

