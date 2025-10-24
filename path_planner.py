"""
路径规划模块
实现A*算法进行全局路径规划
"""

import heapq
import math
import numpy as np
import config


class AStarNode:
    """A*算法节点"""
    
    def __init__(self, x, y, g_cost=0, h_cost=0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # 从起点到当前节点的实际代价
        self.h_cost = h_cost  # 从当前节点到终点的启发式代价
        self.f_cost = g_cost + h_cost  # 总代价
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


class PathPlanner:
    """路径规划器"""
    
    def __init__(self, grid_map):
        self.grid_map = grid_map

    def a_star_safe(self, start_grid, goal_grid, expansion_radius=None, allow_diagonal=True):
        """安全的A*路径规划（使用膨胀地图避免碰撞）

        Args:
            start_grid: 起点栅格坐标 (x, y)
            goal_grid: 终点栅格坐标 (x, y)
            expansion_radius: 障碍物膨胀半径（格子数），None则使用config.OBSTACLE_EXPANSION
            allow_diagonal: 是否允许对角线移动

        Returns:
            路径列表 [(x1, y1), (x2, y2), ...] 或 None
        """
        if expansion_radius is None:
            expansion_radius = config.OBSTACLE_EXPANSION

        # 保存原始地图
        original_map = self.grid_map.grid_map.copy()

        try:
            # 使用膨胀地图进行路径规划
            expanded_map = self.grid_map.expand_obstacles(expansion_radius)

            # 确保起点和终点在膨胀地图中仍然是FREE
            # 否则机器人当前位置可能被膨胀的障碍物覆盖
            start_x, start_y = start_grid
            goal_x, goal_y = goal_grid

            # 保护起点周围的区域（膨胀半径+2的范围）
            protect_radius = expansion_radius + 2
            for dy in range(-protect_radius, protect_radius + 1):
                for dx in range(-protect_radius, protect_radius + 1):
                    nx, ny = start_x + dx, start_y + dy
                    if self.grid_map.is_valid_grid(nx, ny):
                        # 只要原始地图中不是障碍物，就标记为FREE
                        if original_map[ny, nx] != config.MAP_STATE_OCCUPIED:
                            expanded_map[ny, nx] = config.MAP_STATE_FREE

            # 保护终点周围的区域（膨胀半径+2的范围）
            for dy in range(-protect_radius, protect_radius + 1):
                for dx in range(-protect_radius, protect_radius + 1):
                    nx, ny = goal_x + dx, goal_y + dy
                    if self.grid_map.is_valid_grid(nx, ny):
                        # 只要原始地图中不是障碍物，就标记为FREE
                        if original_map[ny, nx] != config.MAP_STATE_OCCUPIED:
                            expanded_map[ny, nx] = config.MAP_STATE_FREE

            # 临时使用膨胀地图
            self.grid_map.grid_map = expanded_map

            # 调用标准A*算法
            path = self.a_star(start_grid, goal_grid, allow_diagonal)

            return path
        finally:
            # 恢复原始地图
            self.grid_map.grid_map = original_map

    def a_star(self, start_grid, goal_grid, allow_diagonal=True):
        """A*路径规划
        
        Args:
            start_grid: 起点栅格坐标 (x, y)
            goal_grid: 终点栅格坐标 (x, y)
            allow_diagonal: 是否允许对角线移动
            
        Returns:
            路径列表 [(x1, y1), (x2, y2), ...] 或 None
        """
        start_x, start_y = start_grid
        goal_x, goal_y = goal_grid
        
        # 检查起点和终点是否有效
        if not self._is_valid_position(start_x, start_y):
            # 起点无效，可能是坐标越界
            if start_x < 0 or start_y < 0:
                # 坐标为负数，强制设置为0
                start_x = max(0, start_x)
                start_y = max(0, start_y)
                print(f" 起点坐标为负数，已修正为: ({start_x}, {start_y})")
            elif start_x >= self.grid_map.grid_col_count or start_y >= self.grid_map.grid_row_count:
                # 坐标超出地图范围
                start_x = min(start_x, self.grid_map.grid_col_count - 1)
                start_y = min(start_y, self.grid_map.grid_row_count - 1)
                print(f" 起点坐标超出范围，已修正为: ({start_x}, {start_y})")
            else:
                # 起点在障碍物上
                # print(f"起点在障碍物上: ({start_x}, {start_y})")  # 注释掉，避免刷屏
                return None

        if not self._is_valid_position(goal_x, goal_y):
            # 终点无效
            if goal_x < 0 or goal_y < 0:
                goal_x = max(0, goal_x)
                goal_y = max(0, goal_y)
            elif goal_x >= self.grid_map.grid_col_count or goal_y >= self.grid_map.grid_row_count:
                goal_x = min(goal_x, self.grid_map.grid_col_count - 1)
                goal_y = min(goal_y, self.grid_map.grid_row_count - 1)
            else:
                # print(f"终点无效: ({goal_x}, {goal_y})")  # 注释掉，避免刷屏
                return None
        
        # 初始化开放列表和关闭列表
        open_list = []
        closed_set = set()
        
        # 创建起始节点
        start_node = AStarNode(
            start_x, start_y,
            g_cost=0,
            h_cost=self._heuristic(start_x, start_y, goal_x, goal_y)
        )
        
        heapq.heappush(open_list, start_node)
        
        # 用于快速查找节点
        open_dict = {(start_x, start_y): start_node}
        
        # 定义移动方向
        if allow_diagonal:
            # 8方向移动
            directions = [
                (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),  # 上下左右
                (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)  # 对角线
            ]
        else:
            # 4方向移动
            directions = [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0)]
        
        # A*主循环 - 优化版本，限制搜索次数
        max_iterations = 5000  # 限制最大搜索次数
        iterations = 0
        
        while open_list and iterations < max_iterations:
            iterations += 1
            
            # 取出f_cost最小的节点
            current = heapq.heappop(open_list)
            current_pos = (current.x, current.y)
            
            # 从open_dict中移除
            if current_pos in open_dict:
                del open_dict[current_pos]
            
            # 检查是否到达目标（允许一定误差）
            if abs(current.x - goal_x) <= 2 and abs(current.y - goal_y) <= 2:
                return self._reconstruct_path(current)
            
            # 加入关闭列表
            closed_set.add(current_pos)
            
            # 遍历邻居
            for dx, dy, cost in directions:
                neighbor_x = current.x + dx
                neighbor_y = current.y + dy
                neighbor_pos = (neighbor_x, neighbor_y)
                
                # 检查是否在关闭列表中
                if neighbor_pos in closed_set:
                    continue
                
                # 检查是否可通行
                if not self._is_valid_position(neighbor_x, neighbor_y):
                    continue
                
                # 对角线移动时检查两侧是否有障碍物
                if allow_diagonal and abs(dx) + abs(dy) == 2:
                    if not self._is_valid_position(current.x + dx, current.y):
                        continue
                    if not self._is_valid_position(current.x, current.y + dy):
                        continue
                
                # 计算新的g_cost
                new_g_cost = current.g_cost + cost
                
                # 检查是否在开放列表中
                if neighbor_pos in open_dict:
                    neighbor_node = open_dict[neighbor_pos]
                    if new_g_cost < neighbor_node.g_cost:
                        # 找到更好的路径，更新节点
                        neighbor_node.g_cost = new_g_cost
                        neighbor_node.f_cost = new_g_cost + neighbor_node.h_cost
                        neighbor_node.parent = current
                        # 优化：不重新堆化，直接标记需要更新
                else:
                    # 创建新节点
                    h_cost = self._heuristic(neighbor_x, neighbor_y, goal_x, goal_y)
                    neighbor_node = AStarNode(
                        neighbor_x, neighbor_y,
                        g_cost=new_g_cost,
                        h_cost=h_cost,
                        parent=current
                    )
                    heapq.heappush(open_list, neighbor_node)
                    open_dict[neighbor_pos] = neighbor_node
        
        # 没有找到路径
        # print("未找到路径")  # 注释掉，避免刷屏
        return None
    
    def _heuristic(self, x1, y1, x2, y2):
        """启发式函数（欧几里得距离）"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def _is_valid_position(self, x, y):
        """检查位置是否有效且可通行"""
        # 检查边界
        if not (0 <= x < self.grid_map.grid_col_count and
                0 <= y < self.grid_map.grid_row_count):
            return False

        # 检查是否是障碍物
        cell_state = self.grid_map.grid_map[y, x]
        return cell_state != config.MAP_STATE_OCCUPIED
    
    def _reconstruct_path(self, node):
        """重建路径"""
        path = []
        current = node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        path.reverse()
        return path
    
    def smooth_path(self, path, target_distance_cells: int = 7, min_distance_cells: int = 2):
        """
        高级路径平滑（从planning.py改进）

        策略：
        1. 尽可能接近target_distance的路径点间距
        2. 转弯处允许使用较短距离
        3. 确保连线不穿墙

        Args:
            path: 原始路径 [(x1,y1), (x2,y2), ...]
            target_distance_cells: 期望的相邻路径点距离（格子数）
            min_distance_cells: 最小允许距离（转弯处）
        """
        if not path or len(path) <= 2:
            return path

        smoothed = [path[0]]  # 起点必须保留
        current_idx = 0

        while current_idx < len(path) - 1:
            best_next_idx = self._find_best_next_waypoint(
                path, current_idx, target_distance_cells, min_distance_cells
            )
            smoothed.append(path[best_next_idx])
            current_idx = best_next_idx

        # 确保终点被包含
        if smoothed[-1] != path[-1]:
            smoothed.append(path[-1])

        return smoothed

    def _find_best_next_waypoint(self, path, current_idx, target_dist, min_dist):
        """
        从当前点开始，找到最佳的下一个路径点

        优先级：
        1. 尽可能接近target_distance的点
        2. 转弯处允许使用较短距离
        3. 确保连线不穿墙
        """
        current_pos = path[current_idx]
        best_idx = current_idx + 1  # 最差情况下选择下一个点
        best_score = float('inf')

        max_search_range = min(current_idx + target_dist * 2, len(path) - 1)

        for candidate_idx in range(current_idx + 1, max_search_range + 1):
            candidate_pos = path[candidate_idx]
            distance = self._pixel_distance(current_pos, candidate_pos)

            # 距离太小，跳过（除非是最后一个点）
            if distance < min_dist and candidate_idx < len(path) - 1:
                continue

            # 检查是否穿墙
            if not self._is_line_clear(current_pos, candidate_pos):
                continue

            # 计算得分（距离越接近target_dist越好）
            distance_score = abs(distance - target_dist)

            if distance_score < best_score:
                best_score = distance_score
                best_idx = candidate_idx

            # 如果找到了接近目标距离的点，可以早期退出
            if distance >= target_dist * 0.9 and distance <= target_dist * 1.1:
                break

        return best_idx

    def _pixel_distance(self, pos1, pos2):
        """计算两个栅格点之间的欧几里德距离"""
        import math
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
    
    def _is_line_clear(self, start, end):
        """检查两点之间的直线是否无障碍"""
        x0, y0 = start
        x1, y1 = end
        
        # Bresenham直线算法
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if not self._is_valid_position(x, y):
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not self._is_valid_position(x, y):
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        
        return self._is_valid_position(x1, y1)
    
    def find_nearest_frontier(self, robot_grid_pos, frontiers):
        """找到最近的前沿点"""
        if not frontiers:
            return None
        
        min_dist = float('inf')
        nearest = None
        
        for frontier in frontiers:
            dist = self._heuristic(robot_grid_pos[0], robot_grid_pos[1], 
                                  frontier[0], frontier[1])
            if dist < min_dist:
                min_dist = dist
                nearest = frontier
        
        return nearest

