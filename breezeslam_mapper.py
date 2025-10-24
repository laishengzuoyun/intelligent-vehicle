"""
BreezeSLAM集成模块 - 高效的SLAM建图算法
"""

import numpy as np
import time
from breezyslam.sensors import Laser
from breezyslam.algorithms import RMHC_SLAM
import config

class BreezeSLAMMapper:
    """BreezeSLAM建图器 - 高效的SLAM算法实现"""
    
    def __init__(self, map_size_pixels=500, map_size_meters=50):
        """
        初始化BreezeSLAM建图器 - 优化版本
        
        Args:
            map_size_pixels: 地图像素大小 (500x500)
            map_size_meters: 地图实际大小 (50m x 50m)
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        
        # 创建激光雷达配置 - 高性能版本
        # 进一步优化参数，提高建图速度
        self.laser = Laser(
            scan_size=72,  # 从180减少到72，每5度一个点，大幅减少计算量
            scan_rate_hz=10,  # 提高到10Hz，但减少点数
            detection_angle_degrees=360,
            distance_no_detection_mm=4000,  # 减少到4米，减少计算范围
            detection_margin=50,  # 增加边距，减少噪声
            offset_mm=0
        )
        
        # 创建SLAM算法 - 优化参数
        self.slam = RMHC_SLAM(
            self.laser,
            map_size_pixels,
            map_size_meters
        )
        
        # 优化SLAM参数 - 高性能版本
        self.slam.map_quality = 20  # 进一步降低质量要求，提高速度
        self.slam.hole_width_mm = 800  # 增加孔洞宽度，提高成功率
        
        # 机器人位姿
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        
        # 性能统计
        self.update_count = 0
        self.last_update_time = 0
        self.processing_times = []
        
        print(f"BreezeSLAM建图器初始化完成")
        print(f"  - 地图大小: {map_size_pixels}x{map_size_pixels} 像素")
        print(f"  - 实际大小: {map_size_meters}x{map_size_meters} 米")
        print(f"  - 扫描点数: {self.laser.scan_size}")
    
    def update_map(self, robot_x, robot_y, robot_theta, lidar_data):
        """
        使用BreezeSLAM更新地图 - 优化版本
        
        Args:
            robot_x, robot_y: 机器人世界坐标 (mm)
            robot_theta: 机器人朝向 (度)
            lidar_data: 雷达数据列表 [(angle, distance), ...]
        """
        start_time = time.time()
        
        # 限制更新频率
        current_time = time.time() * 1000
        if current_time - self.last_update_time < config.LIDAR_PROCESSING_INTERVAL:
            return None
        
        self.last_update_time = current_time
        
        # 检查雷达数据
        if not lidar_data or len(lidar_data) == 0:
            return None
        
        # 数据质量检查
        if not self._check_data_quality(lidar_data):
            return None
        
        # 更新机器人位姿 (转换为米)
        self.robot_pose = [
            robot_x / 1000.0,  # mm -> m
            robot_y / 1000.0,  # mm -> m
            np.radians(robot_theta)  # 度 -> 弧度
        ]
        
        # 执行SLAM更新
        try:
            # BreezeSLAM需要距离和角度列表
            distances, angles = self._extract_distances_angles(lidar_data)
            
            if len(distances) < 50:  # 至少需要50个点
                return None
            
            # 执行SLAM更新
            self.slam.update(distances, scan_angles_degrees=angles)
            self.update_count += 1
            
            # 记录处理时间
            processing_time = (time.time() - start_time) * 1000
            self.processing_times.append(processing_time)
            
            # 保持最近100次的处理时间
            if len(self.processing_times) > 100:
                self.processing_times = self.processing_times[-100:]
            
            # 获取更新后的地图
            map_bytes = bytearray(self.map_size_pixels * self.map_size_pixels)
            self.slam.getmap(map_bytes)
            
            return self._convert_map_to_grid(map_bytes)
            
        except Exception as e:
            print(f"BreezeSLAM更新失败: {e}")
            return None
    
    def _check_data_quality(self, lidar_data):
        """
        检查雷达数据质量 - 高性能版本
        
        Args:
            lidar_data: 雷达数据列表
            
        Returns:
            bool: 数据质量是否合格
        """
        if len(lidar_data) < 5:  # 进一步降低最小数据点要求
            return False
        
        # 简化数据质量检查，只检查基本有效性
        valid_count = 0
        for angle_rad, distance_mm in lidar_data:
            if 100 <= distance_mm <= 4000:  # 合理的距离范围，减少检查范围
                valid_count += 1
        
        # 至少20%的数据有效即可，进一步降低要求
        valid_ratio = valid_count / len(lidar_data)
        if valid_ratio < 0.2:
            return False
        
        return True
    
    def _extract_distances_angles(self, lidar_data):
        """
        从雷达数据中提取距离和角度列表 - 优化版本
        
        Args:
            lidar_data: 原始雷达数据 [(angle, distance), ...]
            
        Returns:
            (distances, angles) 元组
        """
        if not lidar_data or len(lidar_data) == 0:
            return [], []
        
        distances = []
        angles = []
        
        # 简化数据预处理
        valid_data = []
        for angle_rad, distance_mm in lidar_data:
            # 过滤无效距离
            if 100 <= distance_mm <= 4000:  # 100mm到4m的有效范围，减少计算量
                valid_data.append((angle_rad, distance_mm))
        
        # 如果有效数据太少，返回空
        if len(valid_data) < 5:  # 进一步降低最小数据要求
            return [], []
        
        # 按角度排序，确保数据有序
        valid_data.sort(key=lambda x: x[0])
        
        # 直接使用有效数据，不进行插值
        for angle_rad, distance_mm in valid_data:
            # 角度转换：雷达数据是局部角度（相对于机器人朝向）
            # 需要转换为全局角度（相对于世界坐标系）
            angle_deg = np.degrees(angle_rad)
            
            # 雷达正方向是前方（θ=0°），需要转换为BreezeSLAM期望的角度
            # 雷达数据：0°=前方，90°=右方（顺时针）
            # 机器人位姿：0°=前方，90°=左方（逆时针）
            # BreezeSLAM期望：0°=前方，90°=右方，180°=后方，270°=左方
            # 雷达和机器人坐标系相反，需要取反角度
            angle_deg = -angle_deg
            
            # 确保角度在0-360范围内
            if angle_deg < 0:
                angle_deg += 360
            elif angle_deg >= 360:
                angle_deg -= 360
            
            # 限制距离范围
            distance_mm = max(100, min(distance_mm, 4000))  # 限制在100-4000mm范围
            
            distances.append(int(distance_mm))
            angles.append(angle_deg)
        
        return distances, angles
    
    def _interpolate_lidar_data(self, lidar_data):
        """
        对雷达数据进行插值，填充缺失的角度
        
        Args:
            lidar_data: 原始雷达数据 [(angle, distance), ...]
            
        Returns:
            插值后的雷达数据
        """
        if len(lidar_data) < 2:
            return lidar_data
        
        interpolated = []
        
        # 按角度排序
        sorted_data = sorted(lidar_data, key=lambda x: x[0])
        
        for i in range(len(sorted_data)):
            current_angle, current_distance = sorted_data[i]
            next_angle, next_distance = sorted_data[(i + 1) % len(sorted_data)]
            
            # 添加当前点
            interpolated.append((current_angle, current_distance))
            
            # 计算角度差
            angle_diff = next_angle - current_angle
            if angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            elif angle_diff < -np.pi:
                angle_diff += 2 * np.pi
            
            # 如果角度差太大，进行插值
            if abs(angle_diff) > np.pi / 180:  # 大于1度
                steps = int(abs(angle_diff) / (np.pi / 180))  # 每1度插值
                if steps > 1:
                    for j in range(1, steps):
                        t = j / steps
                        interp_angle = current_angle + t * angle_diff
                        interp_distance = current_distance + t * (next_distance - current_distance)
                        interpolated.append((interp_angle, interp_distance))
        
        return interpolated
    
    def _convert_map_to_grid(self, map_bytes):
        """
        将BreezeSLAM地图转换为栅格地图格式 - 优化版本
        
        Args:
            map_bytes: BreezeSLAM地图字节数据
            
        Returns:
            栅格地图数组
        """
        # 将字节数据转换为numpy数组
        map_array = np.frombuffer(map_bytes, dtype=np.uint8)
        map_array = map_array.reshape((self.map_size_pixels, self.map_size_pixels))
        
        # 转换为我们的地图格式
        grid_map = np.full((self.map_size_pixels, self.map_size_pixels), 
                          config.MAP_STATE_UNKNOWN, dtype=np.int8)
        
        # 转换地图值 - 优化映射
        # BreezeSLAM: 0=未知, 255=空闲, 128=障碍物
        # 我们的格式: -1=未知, 0=空闲, 1=障碍物
        
        # 优化阈值设置，提高建图质量
        free_threshold = 150  # 进一步降低阈值，识别更多空闲区域
        occupied_threshold = 100  # 降低阈值，识别更多障碍物
        
        grid_map[map_array > free_threshold] = config.MAP_STATE_FREE
        grid_map[map_array < occupied_threshold] = config.MAP_STATE_OCCUPIED
        
        # 简化地图处理，直接返回
        return grid_map
    
    def _apply_morphology(self, grid_map):
        """
        应用形态学操作改善地图质量
        
        Args:
            grid_map: 原始栅格地图
            
        Returns:
            处理后的栅格地图
        """
        # 简单的形态学操作：去除小的噪声点
        processed_map = grid_map.copy()
        
        # 去除小的障碍物点（可能是噪声）
        for i in range(1, grid_map.shape[0] - 1):
            for j in range(1, grid_map.shape[1] - 1):
                if grid_map[i, j] == config.MAP_STATE_OCCUPIED:
                    # 检查周围8个点
                    neighbors = [
                        grid_map[i-1, j-1], grid_map[i-1, j], grid_map[i-1, j+1],
                        grid_map[i, j-1], grid_map[i, j+1],
                        grid_map[i+1, j-1], grid_map[i+1, j], grid_map[i+1, j+1]
                    ]
                    
                    # 如果周围障碍物太少，认为是噪声
                    occupied_neighbors = sum(1 for n in neighbors if n == config.MAP_STATE_OCCUPIED)
                    if occupied_neighbors < 2:
                        processed_map[i, j] = config.MAP_STATE_UNKNOWN
        
        return processed_map
    
    def _apply_smoothing(self, grid_map):
        """
        应用平滑滤波减少地图噪声
        
        Args:
            grid_map: 原始栅格地图
            
        Returns:
            平滑后的栅格地图
        """
        smoothed_map = grid_map.copy()
        
        # 对地图进行平滑处理
        for i in range(1, grid_map.shape[0] - 1):
            for j in range(1, grid_map.shape[1] - 1):
                # 获取3x3邻域
                neighborhood = grid_map[i-1:i+2, j-1:j+2]
                
                # 计算邻域中各类别的数量
                free_count = np.sum(neighborhood == config.MAP_STATE_FREE)
                occupied_count = np.sum(neighborhood == config.MAP_STATE_OCCUPIED)
                unknown_count = np.sum(neighborhood == config.MAP_STATE_UNKNOWN)
                
                # 如果中心点是未知区域，根据邻域信息进行推断
                if grid_map[i, j] == config.MAP_STATE_UNKNOWN:
                    if free_count >= 5:  # 如果周围大部分是空闲区域
                        smoothed_map[i, j] = config.MAP_STATE_FREE
                    elif occupied_count >= 5:  # 如果周围大部分是障碍物
                        smoothed_map[i, j] = config.MAP_STATE_OCCUPIED
                
                # 如果中心点是障碍物，但周围大部分是空闲区域，可能是噪声
                elif grid_map[i, j] == config.MAP_STATE_OCCUPIED and free_count >= 6:
                    smoothed_map[i, j] = config.MAP_STATE_FREE
                
                # 如果中心点是空闲区域，但周围大部分是障碍物，可能是噪声
                elif grid_map[i, j] == config.MAP_STATE_FREE and occupied_count >= 6:
                    smoothed_map[i, j] = config.MAP_STATE_OCCUPIED
        
        return smoothed_map
    
    def get_debug_info(self):
        """获取调试信息"""
        return {
            'update_count': self.update_count,
            'laser_scan_size': self.laser.scan_size,
            'map_size_pixels': self.map_size_pixels,
            'map_size_meters': self.map_size_meters,
            'robot_pose': self.robot_pose,
            'slam_map_quality': getattr(self.slam, 'map_quality', 'unknown'),
            'slam_hole_width': getattr(self.slam, 'hole_width_mm', 'unknown')
        }
    
    def reset_with_new_params(self, map_quality=50, hole_width_mm=200):
        """使用新参数重置建图器"""
        try:
            self.slam = RMHC_SLAM(
                self.laser,
                self.map_size_pixels,
                self.map_size_meters
            )
            self.slam.map_quality = map_quality
            self.slam.hole_width_mm = hole_width_mm
            self.robot_pose = [0.0, 0.0, 0.0]
            self.update_count = 0
            self.processing_times = []
            print(f"BreezeSLAM建图器已重置，新参数: map_quality={map_quality}, hole_width={hole_width_mm}mm")
        except Exception as e:
            print(f"重置BreezeSLAM失败: {e}")
    
    def get_performance_stats(self):
        """获取性能统计信息"""
        if not self.processing_times:
            return "暂无性能数据"
        
        avg_time = np.mean(self.processing_times)
        max_time = np.max(self.processing_times)
        min_time = np.min(self.processing_times)
        
        return {
            'update_count': self.update_count,
            'avg_processing_time_ms': avg_time,
            'max_processing_time_ms': max_time,
            'min_processing_time_ms': min_time,
            'updates_per_second': 1000.0 / avg_time if avg_time > 0 else 0
        }
    
    def reset(self):
        """重置建图器"""
        self.slam = RMHC_SLAM(
            self.laser,
            self.map_size_pixels,
            self.map_size_meters
        )
        self.robot_pose = [0.0, 0.0, 0.0]
        self.update_count = 0
        self.processing_times = []
        print("BreezeSLAM建图器已重置")


# 测试函数
def test_breezeslam():
    """测试BreezeSLAM功能 - 优化版本"""
    print("测试BreezeSLAM建图器...")
    
    # 创建建图器
    mapper = BreezeSLAMMapper()
    
    # 模拟更真实的雷达数据 - 创建简单环境
    lidar_data = []
    
    # 创建简单的方形环境 - 优化版本
    for i in range(180):  # 180个扫描点，2度分辨率，平衡精度和性能
        angle = i * 2 * np.pi / 180  # 转换为弧度
        
        # 简单的距离计算 - 创建方形环境
        robot_x, robot_y = 2500, 2500  # 机器人位置
        
        # 简单的方形边界
        if abs(np.sin(angle)) > 0.01:
            dist_y = 2000 / abs(np.sin(angle))  # 到y方向边界的距离
        else:
            dist_y = 6000
            
        if abs(np.cos(angle)) > 0.01:
            dist_x = 2000 / abs(np.cos(angle))  # 到x方向边界的距离
        else:
            dist_x = 6000
        
        distance = min(dist_x, dist_y)
        distance = max(100, min(distance, 6000))  # 限制在100-6000mm范围
        
        lidar_data.append((angle, distance))
    
    # 测试地图更新
    robot_x, robot_y, robot_theta = 2000, 2000, 0  # mm, mm, 度
    
    print(f"测试数据: {len(lidar_data)}个雷达点")
    print(f"机器人位置: ({robot_x}, {robot_y}), 朝向: {robot_theta}度")
    
    start_time = time.time()
    grid_map = mapper.update_map(robot_x, robot_y, robot_theta, lidar_data)
    end_time = time.time()
    
    if grid_map is not None:
        print("地图更新成功!")
        print(f"  - 地图大小: {grid_map.shape}")
        print(f"  - 处理时间: {(end_time - start_time)*1000:.2f}ms")
        print(f"  - 未知区域: {np.sum(grid_map == config.MAP_STATE_UNKNOWN)}")
        print(f"  - 空闲区域: {np.sum(grid_map == config.MAP_STATE_FREE)}")
        print(f"  - 障碍物: {np.sum(grid_map == config.MAP_STATE_OCCUPIED)}")
        
        # 性能统计
        stats = mapper.get_performance_stats()
        print(f"  - 性能统计: {stats}")
        
        # 调试信息
        debug_info = mapper.get_debug_info()
        print(f"  - 调试信息: {debug_info}")
        
        # 测试多次更新
        print("\n测试多次更新...")
        for i in range(5):
            # 稍微移动机器人位置
            new_x = robot_x + i * 100
            new_y = robot_y + i * 50
            new_theta = i * 10
            
            grid_map = mapper.update_map(new_x, new_y, new_theta, lidar_data)
            if grid_map is not None:
                print(f"  更新 {i+1}: 成功")
            else:
                print(f"  更新 {i+1}: 失败")
        
    else:
        print("地图更新失败")
        print("可能的原因:")
        print("  - 雷达数据质量不足")
        print("  - BreezeSLAM参数配置问题")
        print("  - 数据格式不匹配")


if __name__ == "__main__":
    test_breezeslam()
