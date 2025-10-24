"""
验收考试导航模块
实现目标导向的探索和精确单元格导航
"""

import math
import numpy as np
import config


class ExamNavigator:
    """验收考试导航器"""
    
    def __init__(self, map_manager, path_planner):
        self.map_manager = map_manager
        self.path_planner = path_planner
        
        # 验收配置
        self.grid_size = 4  # 4×4方格
        self.cell_size = 700  # 每个单元格70cm = 700mm
        
        # 入口和出口（验收前一天公布后设置）
        self.entrance_cell = (0, 0)
        self.exit_cell = (3, 2)
        
        # 当前阶段
        self.stage = "idle"  # idle, to_exit, to_entrance
        
    def set_entrance_exit(self, entrance, exit_cell):
        """设置入口和出口坐标
        
        Args:
            entrance: 入口单元格 (grid_x, grid_y)
            exit_cell: 出口单元格 (grid_x, grid_y)
        """
        self.entrance_cell = entrance
        self.exit_cell = exit_cell
        print(f" 设置入口: {entrance}")
        print(f" 设置出口: {exit_cell}")
    
    def cell_to_world(self, cell):
        """将单元格坐标转换为世界坐标（中心点）
        
        Args:
            cell: 单元格坐标 (grid_x, grid_y)
            
        Returns:
            世界坐标 (x_mm, y_mm)
        """
        grid_x, grid_y = cell
        world_x = grid_x * self.cell_size + self.cell_size / 2
        world_y = grid_y * self.cell_size + self.cell_size / 2
        return (world_x, world_y)
    
    def world_to_cell(self, world_pos):
        """将世界坐标转换为单元格坐标
        
        Args:
            world_pos: 世界坐标 (x_mm, y_mm)
            
        Returns:
            单元格坐标 (grid_x, grid_y)
        """
        x, y = world_pos
        grid_x = int(x / self.cell_size)
        grid_y = int(y / self.cell_size)
        return (grid_x, grid_y)
    
    def is_in_cell(self, world_pos, cell):
        """检查位置是否在指定单元格内
        
        Args:
            world_pos: 世界坐标 (x_mm, y_mm)
            cell: 单元格坐标 (grid_x, grid_y)
            
        Returns:
            是否在单元格内
        """
        x, y = world_pos
        cell_x, cell_y = cell
        
        # 单元格边界（留有10mm余量）
        margin = 10
        min_x = cell_x * self.cell_size + margin
        max_x = (cell_x + 1) * self.cell_size - margin
        min_y = cell_y * self.cell_size + margin
        max_y = (cell_y + 1) * self.cell_size - margin
        
        return min_x <= x <= max_x and min_y <= y <= max_y
    
    def find_frontier_toward_goal(self, current_pos, goal_cell):
        """查找朝向目标的前沿点
        
        Args:
            current_pos: 当前位置 (x_mm, y_mm)
            goal_cell: 目标单元格 (grid_x, grid_y)
            
        Returns:
            最佳前沿点 (x_mm, y_mm) 或 None
        """
        # 获取所有前沿点
        frontiers = self.map_manager.get_frontiers()
        
        if not frontiers:
            return None
        
        # 目标位置（世界坐标）
        goal_pos = self.cell_to_world(goal_cell)
        
        # 选择最接近目标方向的前沿点
        best_frontier = None
        min_score = float('inf')
        
        for frontier in frontiers:
            # 计算前沿点到目标的距离
            dist_to_goal = math.sqrt(
                (frontier[0] - goal_pos[0])**2 + 
                (frontier[1] - goal_pos[1])**2
            )
            
            # 计算当前位置到前沿点的距离
            dist_to_frontier = math.sqrt(
                (frontier[0] - current_pos[0])**2 + 
                (frontier[1] - current_pos[1])**2
            )
            
            # 综合评分：70%权重给目标距离，30%权重给可达性
            score = dist_to_goal * 0.7 + dist_to_frontier * 0.3
            
            if score < min_score:
                min_score = score
                best_frontier = frontier
        
        return best_frontier
    
    def can_reach_goal(self, current_pos, goal_cell):
        """检查是否可以直接到达目标
        
        Args:
            current_pos: 当前位置 (x_mm, y_mm)
            goal_cell: 目标单元格 (grid_x, grid_y)
            
        Returns:
            是否可以直接到达
        """
        goal_pos = self.cell_to_world(goal_cell)
        
        # 尝试规划路径
        path = self.path_planner.plan_path(current_pos, goal_pos)
        
        return path is not None and len(path) > 0
    
    def plan_to_cell(self, current_pos, target_cell):
        """规划到指定单元格的路径
        
        Args:
            current_pos: 当前位置 (x_mm, y_mm)
            target_cell: 目标单元格 (grid_x, grid_y)
            
        Returns:
            路径列表 [(x1, y1), (x2, y2), ...] 或 None
        """
        # 目标位置：单元格中心
        target_pos = self.cell_to_world(target_cell)
        
        # 使用A*规划路径
        path = self.path_planner.plan_path(current_pos, target_pos)
        
        return path
    
    def get_next_exploration_target(self, current_pos):
        """获取下一个探索目标
        
        Args:
            current_pos: 当前位置 (x_mm, y_mm)
            
        Returns:
            下一个目标位置 (x_mm, y_mm) 或 None
        """
        if self.stage == "to_exit":
            # 前往出口阶段：选择朝向出口的前沿点
            return self.find_frontier_toward_goal(current_pos, self.exit_cell)
        
        elif self.stage == "to_entrance":
            # 返回入口阶段：直接规划路径
            return self.cell_to_world(self.entrance_cell)
        
        else:
            # 空闲状态：使用传统前沿探索
            frontiers = self.map_manager.get_frontiers()
            if frontiers:
                return self.path_planner.find_nearest_frontier(current_pos, frontiers)
            return None
    
    def start_exam(self):
        """开始验收考试"""
        print("=" * 60)
        print("[EXAM] 开始验收考试")
        print(f"[ENTRANCE] 入口: {self.entrance_cell}")
        print(f"[EXIT] 出口: {self.exit_cell}")
        print("=" * 60)

        self.stage = "to_exit"
        print("[STAGE1] 阶段1：导航到出口")

    def reach_exit(self):
        """到达出口"""
        print(f"[OK] 到达出口 {self.exit_cell}")
        print("=" * 60)
        print("[STAGE2] 阶段2：返回入口")

        self.stage = "to_entrance"

    def reach_entrance(self):
        """到达入口"""
        print(f"[OK] 返回入口 {self.entrance_cell}")
        print("=" * 60)
        print("[COMPLETE] 验收考试完成！")
        print("=" * 60)

        self.stage = "idle"
    
    def get_exam_status(self):
        """获取验收状态
        
        Returns:
            状态字典
        """
        return {
            'stage': self.stage,
            'entrance': self.entrance_cell,
            'exit': self.exit_cell,
            'entrance_world': self.cell_to_world(self.entrance_cell),
            'exit_world': self.cell_to_world(self.exit_cell)
        }
    
    def draw_grid_overlay(self, ax):
        """在地图上绘制单元格网格
        
        Args:
            ax: matplotlib axes对象
        """
        # 绘制网格线
        for i in range(self.grid_size + 1):
            # 垂直线
            x = i * self.cell_size
            ax.plot([x, x], [0, self.grid_size * self.cell_size], 
                   'g--', linewidth=0.5, alpha=0.5)
            
            # 水平线
            y = i * self.cell_size
            ax.plot([0, self.grid_size * self.cell_size], [y, y], 
                   'g--', linewidth=0.5, alpha=0.5)
        
        # 标记入口（绿色）
        entrance_pos = self.cell_to_world(self.entrance_cell)
        ax.plot(entrance_pos[0], entrance_pos[1], 'go', 
               markersize=15, label='入口')
        ax.text(entrance_pos[0], entrance_pos[1] + 50, 'START', 
               ha='center', color='green', fontsize=10, weight='bold')
        
        # 标记出口（红色）
        exit_pos = self.cell_to_world(self.exit_cell)
        ax.plot(exit_pos[0], exit_pos[1], 'ro', 
               markersize=15, label='出口')
        ax.text(exit_pos[0], exit_pos[1] + 50, 'EXIT', 
               ha='center', color='red', fontsize=10, weight='bold')
        
        # 标记单元格坐标
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                center = self.cell_to_world((i, j))
                ax.text(center[0], center[1], f'({i},{j})', 
                       ha='center', va='center', 
                       color='gray', fontsize=8, alpha=0.7)

