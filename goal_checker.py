"""
终点可达性检查器
从db1/planning.py提取并适配

功能：
1. 判断从当前位置到终点是否存在可行路径
2. 使用BFS进行连通性检查（比A*快）
3. 检查终点周围是否为已知的自由空间
"""

from collections import deque
from typing import Tuple, Optional
import config


class GoalReachabilityChecker:
    """
    终点可达性检查器
    
    用途：
    - 在探索过程中定期检查终点是否已可达
    - 一旦可达，立即切换到"前往终点"模式，避免过度探索
    """
    
    def __init__(self,
                 check_radius_cells: int = 5,      # 终点周围需要自由的半径（格子数）
                 path_check_interval: int = 1,     # 路径检查间隔（帧数）
                 free_ratio_threshold: float = 0.7):  # 终点周围自由空间比例阈值
        
        self.check_radius = int(check_radius_cells)
        self.check_interval = int(path_check_interval)
        self.free_ratio_threshold = free_ratio_threshold
        
        self._frame_count = 0
        self._last_check_result = False
    
    def is_goal_reachable(self,
                         grid_map,
                         curr_grid_pos: Tuple[int, int],
                         goal_grid_pos: Tuple[int, int],
                         maze_rect: Optional[Tuple[int, int, int, int]] = None,
                         force_check: bool = False) -> bool:
        """
        检查终点是否可达
        
        Args:
            grid_map: 栅格地图
            curr_grid_pos: 当前栅格位置 (grid_x, grid_y)
            goal_grid_pos: 终点栅格位置 (grid_x, grid_y)
            maze_rect: 有效地图区域 (x0, y0, x1, y1)
            force_check: 是否强制检查（忽略帧间隔）
            
        Returns:
            True: 终点可达，应该直接前往终点
            False: 终点不可达，继续探索
        """
        # 帧间隔控制（减少计算开销）
        self._frame_count += 1
        if not force_check and self._frame_count % self.check_interval != 0:
            return self._last_check_result
        
        H, W = grid_map.shape
        gx, gy = int(goal_grid_pos[0]), int(goal_grid_pos[1])
        
        # 检查1：终点是否在地图范围内
        if not (0 <= gx < W and 0 <= gy < H):
            print(f"⚠️ 终点({gx},{gy})超出地图范围")
            self._last_check_result = False
            return False
        
        # 检查2：终点周围是否为自由空间
        if not self._is_goal_area_free(grid_map, gx, gy):
            # print(f"终点区域未探索或有障碍物")  # 注释掉避免刷屏
            self._last_check_result = False
            return False
        
        # 检查3：从当前位置到终点是否存在路径
        cx, cy = int(curr_grid_pos[0]), int(curr_grid_pos[1])
        
        if self._has_path_bfs(grid_map, (cx, cy), (gx, gy), maze_rect):
            print(f"✅ 终点可达！当前({cx},{cy}) -> 终点({gx},{gy})")
            self._last_check_result = True
            return True
        else:
            # print(f"终点尚不可达，继续探索")  # 注释掉避免刷屏
            self._last_check_result = False
            return False
    
    def _is_goal_area_free(self, grid_map, gx: int, gy: int) -> bool:
        """
        检查终点周围是否为已知的自由空间
        
        策略：
        - 终点本身必须是自由空间
        - 终点周围一定半径内大部分区域应该是自由空间
        """
        H, W = grid_map.shape
        
        # 检查终点本身
        if not (0 <= gx < W and 0 <= gy < H):
            return False
        
        if grid_map[gy, gx] != config.MAP_STATE_FREE:
            return False
        
        # 检查周围区域
        r = self.check_radius
        free_count = 0
        total_count = 0
        
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < W and 0 <= ny < H:
                    total_count += 1
                    if grid_map[ny, nx] == config.MAP_STATE_FREE:
                        free_count += 1
        
        # 至少指定比例的周围区域是自由空间
        if total_count == 0:
            return False
        
        free_ratio = free_count / total_count
        return free_ratio >= self.free_ratio_threshold
    
    def _has_path_bfs(self,
                     grid_map,
                     start: Tuple[int, int],
                     goal: Tuple[int, int],
                     maze_rect: Optional[Tuple[int, int, int, int]]) -> bool:
        """
        BFS检查路径连通性（轻量级版本，比A*快）
        
        Args:
            grid_map: 地图
            start: 起点栅格坐标
            goal: 终点栅格坐标
            maze_rect: 搜索范围限制
            
        Returns:
            True: 存在路径, False: 不存在路径
        """
        H, W = grid_map.shape
        
        # ROI范围
        if maze_rect:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(W - 1, x1)
            y1 = min(H - 1, y1)
        else:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        
        sx, sy = start
        gx, gy = goal
        
        # 边界检查
        if not (x0 <= sx <= x1 and y0 <= sy <= y1):
            return False
        if not (x0 <= gx <= x1 and y0 <= gy <= y1):
            return False
        
        # BFS
        queue = deque([(sx, sy)])
        visited = {(sx, sy)}
        
        # 4连通（简化版，更快）
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        max_iterations = 5000  # 防止无限循环
        iterations = 0
        
        while queue and iterations < max_iterations:
            iterations += 1
            x, y = queue.popleft()
            
            # 到达终点（允许一定误差）
            if abs(x - gx) <= 8 and abs(y - gy) <= 8:
                return True
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                if (nx, ny) in visited:
                    continue
                
                # 范围检查
                if not (x0 <= nx <= x1 and y0 <= ny <= y1):
                    continue
                
                # 可通行检查：自由空间
                if grid_map[ny, nx] != config.MAP_STATE_FREE:
                    continue
                
                visited.add((nx, ny))
                queue.append((nx, ny))
        
        return False

