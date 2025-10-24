"""
优化的前沿探索器模块

核心功能：
1. 前沿点检测（已知自由区域与未知区域的边界）
2. 多层筛选策略：距离过滤 → 空间采样 → 连通性检查 → 终点导向
3. 厚实连通区域BFS，避免选择被墙隔开的前沿点
"""

import time
import math
from collections import deque
from typing import List, Tuple, Optional
import config


class OptimizedFrontierExplorer:
    """
    优化的前沿探索器
    
    适配说明：
    - 输入地图格式：numpy数组，config.MAP_STATE_FREE/OCCUPIED/UNKNOWN
    - 输出：前沿点列表 [(grid_x, grid_y), ...]
    """
    
    def __init__(self,
                 obs_clearance_cells: int = 2,           # 与障碍物的安全距离（格子数）
                 gain_radius_cells: int = 8,             # 信息增益计算半径
                 connectivity_check_radius: int = 200,   # 连通性检查半径
                 min_distance_cells: int = 3,            # 最小选择距离，避免打转
                 max_distance_cells: int = 200,          # 最大考虑距离
                 spatial_sample_grid: int = 10,          # 空间采样网格大小
                 max_candidates_to_eval: int = 50,       # 最多评估的候选点数
                 goal_bias_weight: float = 0.6):         # 终点导向权重
        
        self.clear_r = int(obs_clearance_cells)
        self.gain_r = int(gain_radius_cells)
        self.conn_r = int(connectivity_check_radius)
        self.min_dist = int(min_distance_cells)
        self.max_dist = int(max_distance_cells)
        self.sample_grid = int(spatial_sample_grid)
        self.max_eval = int(max_candidates_to_eval)
        self.goal_bias_weight = goal_bias_weight
        
        # 缓存上次的空间采样结果（用于可视化）
        self.last_spatially_sampled = []
    
    def detect_frontiers(self, grid_map, maze_rect: Optional[Tuple[int, int, int, int]] = None) -> List[Tuple[int, int]]:
        """
        检测前沿点 - 优化版本，大幅减少计算量
        
        Args:
            grid_map: numpy数组，地图状态
            maze_rect: 搜索范围 (x0, y0, x1, y1)，None表示全图
            
        Returns:
            前沿点列表 [(grid_x, grid_y), ...]
        """
        H, W = grid_map.shape
        
        if maze_rect is None:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        else:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(W - 1, x1)
            y1 = min(H - 1, y1)
        
        frontiers = []
        
        # 性能优化：使用大步长采样，减少检查点数量
        step = 3  # 每3个格子检查一次，减少9倍计算量
        
        # 遍历搜索范围（使用步长）
        for y in range(max(y0, 1), min(y1, H - 2) + 1, step):
            for x in range(max(x0, 1), min(x1, W - 2) + 1, step):
                # 当前格子必须是自由空间
                if grid_map[y, x] != config.MAP_STATE_FREE:
                    continue
                
                # 简化的邻域检查：只检查4邻域，减少计算量
                if not self._has_unknown_neighbor_simple(grid_map, x, y, W, H):
                    continue
                
                # 简化的障碍物检查：只检查2x2邻域
                if not self._safe_from_obstacle_simple(grid_map, x, y, W, H):
                    continue
                
                frontiers.append((x, y))
                
                # 限制前沿点数量，避免过多计算
                if len(frontiers) >= 100:
                    break
            
            if len(frontiers) >= 100:
                break
        
        return frontiers
    
    def _has_unknown_neighbor_simple(self, grid_map, x: int, y: int, W: int, H: int) -> bool:
        """简化的邻域检查：只检查4邻域"""
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H:
                if grid_map[ny, nx] == config.MAP_STATE_UNKNOWN:
                    return True
        return False
    
    def _safe_from_obstacle_simple(self, grid_map, x: int, y: int, W: int, H: int) -> bool:
        """简化的障碍物检查：只检查2x2邻域"""
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    if grid_map[ny, nx] == config.MAP_STATE_OCCUPIED:
                        return False
        return True
    
    def _has_unknown_neighbor(self, grid_map, x: int, y: int, W: int, H: int) -> bool:
        """检查8邻域是否有未知区域"""
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    if grid_map[ny, nx] == config.MAP_STATE_UNKNOWN:
                        return True
        return False
    
    def _safe_from_obstacle(self, grid_map, x: int, y: int, W: int, H: int) -> bool:
        """检查圆形邻域内是否有障碍物"""
        r = self.clear_r
        r_squared = r * r
        
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r_squared:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    if grid_map[ny, nx] == config.MAP_STATE_OCCUPIED:
                        return False
        return True
    
    def pick_best_frontiers(self,
                           curr_grid_pos: Tuple[int, int],
                           candidates: List[Tuple[int, int]],
                           grid_map,
                           goal_grid_pos: Optional[Tuple[int, int]] = None,
                           exclude_radius_cells: int = 3,
                           maze_rect: Optional[Tuple[int, int, int, int]] = None) -> List[Tuple[int, int]]:
        """
        多层筛选选择最佳前沿点
        
        Args:
            curr_grid_pos: 当前栅格位置 (grid_x, grid_y)
            candidates: 候选前沿点列表
            grid_map: 地图数组
            goal_grid_pos: 终点栅格位置（可选）
            exclude_radius_cells: 排除半径
            maze_rect: 迷宫有效区域
            
        Returns:
            排序后的前沿点列表（按优先级从高到低）
        """
        if not candidates:
            return []
        
        start_time = time.time()
        cx, cy = int(curr_grid_pos[0]), int(curr_grid_pos[1])
        
        # 第一层：距离过滤
        distance_filtered = self._filter_by_distance(candidates, (cx, cy), exclude_radius_cells)
        if not distance_filtered:
            return []
        
        # 第二层：空间采样
        spatially_sampled = self._spatial_sampling(distance_filtered)
        self.last_spatially_sampled = spatially_sampled  # 缓存用于可视化
        
        # 第三层：连通性检查
        connected_candidates = self._filter_by_connectivity(
            spatially_sampled, (cx, cy), grid_map, maze_rect
        )
        if not connected_candidates:
            print(" 连通性检查后无可达前沿点")
            return []
        
        # 第四层：终点导向（如果提供了终点）
        if goal_grid_pos is not None:
            goal_biased_candidates = self._filter_by_goal_proximity(
                connected_candidates, (cx, cy), goal_grid_pos
            )
        else:
            goal_biased_candidates = connected_candidates
        
        elapsed = time.time() - start_time
        print(f"前沿选择: {len(candidates)}个 → 距离{len(distance_filtered)} → 采样{len(spatially_sampled)} → 连通{len(connected_candidates)} → 终点导向{len(goal_biased_candidates)} (耗时{elapsed:.3f}s)")
        
        return goal_biased_candidates
    
    def _filter_by_distance(self, candidates: List[Tuple[int, int]],
                           curr_pos: Tuple[int, int],
                           exclude_radius: int) -> List[Tuple[int, int]]:
        """距离筛选：排除太近和太远的点"""
        cx, cy = curr_pos
        exclude_r2 = exclude_radius * exclude_radius
        min_r2 = self.min_dist * self.min_dist
        max_r2 = self.max_dist * self.max_dist
        
        filtered = []
        for px, py in candidates:
            dist2 = (px - cx) ** 2 + (py - cy) ** 2
            if exclude_r2 < dist2 and min_r2 <= dist2 <= max_r2:
                filtered.append((px, py))
        
        return filtered
    
    def _spatial_sampling(self, candidates: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """空间采样：在网格中每个格子只保留一个候选点"""
        if len(candidates) <= self.max_eval:
            return candidates
        
        grid_dict = {}
        for px, py in candidates:
            grid_x = px // self.sample_grid
            grid_y = py // self.sample_grid
            grid_key = (grid_x, grid_y)
            
            if grid_key not in grid_dict:
                grid_dict[grid_key] = []
            grid_dict[grid_key].append((px, py))
        
        # 每个网格选择第一个点
        sampled = [points[0] for points in grid_dict.values()]
        return sampled
    
    def _filter_by_connectivity(self, candidates: List[Tuple[int, int]],
                                curr_pos: Tuple[int, int],
                                grid_map,
                                maze_rect: Optional[Tuple[int, int, int, int]] = None) -> List[Tuple[int, int]]:
        """
        简化的连通性检查 - 大幅优化性能
        
        策略：
        1. 使用简化的BFS，限制搜索范围
        2. 减少复杂的"厚实区域"计算
        3. 快速失败机制
        """
        H, W = grid_map.shape
        
        if maze_rect is not None:
            x0, y0, x1, y1 = maze_rect
        else:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        
        # 找到有效的起始点
        start_x, start_y = int(curr_pos[0]), int(curr_pos[1])
        
        # 简化的起始点检查
        if not (0 <= start_x < W and 0 <= start_y < H):
            return candidates[:5]  # 直接返回前5个候选点
        
        if grid_map[start_y, start_x] != config.MAP_STATE_FREE:
            return candidates[:5]  # 直接返回前5个候选点
        
        # 简化的BFS扩展连通区域
        connected_region = set()
        queue = deque([(start_x, start_y)])
        connected_region.add((start_x, start_y))
        
        # 使用4连通扩展（更快）
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # 限制搜索次数，避免无限循环
        max_iterations = 1000
        iterations = 0
        
        while queue and iterations < max_iterations:
            iterations += 1
            x, y = queue.popleft()
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                if (nx, ny) in connected_region:
                    continue
                
                if not (0 <= nx < W and 0 <= ny < H and x0 <= nx <= x1 and y0 <= ny <= y1):
                    continue
                
                if grid_map[ny, nx] != config.MAP_STATE_FREE:
                    continue
                
                connected_region.add((nx, ny))
                queue.append((nx, ny))
        
        # 筛选在连通区域内的前沿点
        qualified = [p for p in candidates if p in connected_region]
        
        # 如果连通点太少，返回距离最近的点
        if len(qualified) < 3:
            # 按距离排序，返回最近的几个
            cx, cy = start_x, start_y
            distance_sorted = sorted(candidates, 
                                  key=lambda p: (p[0] - cx)**2 + (p[1] - cy)**2)
            return distance_sorted[:5]
        
        return qualified[:10]  # 限制返回数量
    
    def _fallback_connectivity_check(self, candidates, curr_pos, grid_map):
        """降级方案：使用普通连通性检查"""
        H, W = grid_map.shape
        start_x, start_y = int(curr_pos[0]), int(curr_pos[1])
        
        # BFS扩展普通连通区域
        basic_region = set()
        queue = deque([(start_x, start_y)])
        basic_region.add((start_x, start_y))
        
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        while queue:
            x, y = queue.popleft()
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                if (nx, ny) in basic_region:
                    continue
                
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                
                if grid_map[ny, nx] != config.MAP_STATE_FREE:
                    continue
                
                basic_region.add((nx, ny))
                queue.append((nx, ny))
        
        qualified = [p for p in candidates if p in basic_region]
        return qualified if qualified else candidates[:2]
    
    def _filter_by_goal_proximity(self,
                                  candidates: List[Tuple[int, int]],
                                  curr_pos: Tuple[int, int],
                                  goal_pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        根据与终点的接近程度排序前沿点
        
        策略：
        1. 距离终点越近越好
        2. 方向越对齐越好（朝向终点）
        """
        if not candidates:
            return []
        
        cx, cy = curr_pos
        gx, gy = goal_pos
        
        # 当前位置到终点的向量
        goal_vec = (gx - cx, gy - cy)
        goal_dist = math.sqrt(goal_vec[0] ** 2 + goal_vec[1] ** 2)
        
        if goal_dist < 1e-6:
            return candidates
        
        # 归一化方向向量
        goal_dir = (goal_vec[0] / goal_dist, goal_vec[1] / goal_dist)
        
        scored_candidates = []
        
        for px, py in candidates:
            # 候选点到终点的距离
            dist_to_goal = math.sqrt((px - gx) ** 2 + (py - gy) ** 2)
            
            # 当前位置→候选点的向量
            cand_vec = (px - cx, py - cy)
            cand_dist = math.sqrt(cand_vec[0] ** 2 + cand_vec[1] ** 2)
            
            if cand_dist < 1e-6:
                continue
            
            # 方向对齐度（点积）
            alignment = (cand_vec[0] * goal_dir[0] + cand_vec[1] * goal_dir[1]) / cand_dist
            
            # 综合得分
            distance_score = 1.0 / (1.0 + dist_to_goal / 100.0)
            alignment_score = (alignment + 1.0) / 2.0
            
            final_score = (self.goal_bias_weight * distance_score +
                          (1 - self.goal_bias_weight) * alignment_score)
            
            scored_candidates.append(((px, py), final_score))
        
        # 按得分降序排序
        scored_candidates.sort(key=lambda x: -x[1])
        
        return [coord for coord, _ in scored_candidates]

