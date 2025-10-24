"""
迷宫ROI（感兴趣区域）估计器
从db1/planning.py提取并适配

功能：
1. 估计迷宫的有效矩形边界
2. 避免在整个5000x5000地图上搜索前沿点
3. 只增不减策略，避免边界抖动
"""

from typing import Optional, Tuple
import config


class MapRectifier:
    """
    迷宫有效矩形估计器
    
    策略：
    - 扫描已知障碍物的包围盒
    - 外扩margin并裁剪到地图范围
    - 默认"只增不减"，避免因观测瞬时缺失而收缩抖动
    """
    
    def __init__(self,
                 margin_cells: int = 20,           # ROI在四周外扩的格子边距
                 min_width_cells: int = 5,         # 过滤掉过窄的盒子
                 min_height_cells: int = 5,
                 min_area_cells: int = 400,        # 最小面积过滤
                 allow_shrink: bool = False,       # 是否允许ROI收缩
                 max_shrink_per_update: int = 4):  # 若允许收缩，每次边界最多内缩格子数
        
        self.rect: Optional[Tuple[int, int, int, int]] = None  # (x0, y0, x1, y1)
        self.margin = int(margin_cells)
        self.min_w = int(min_width_cells)
        self.min_h = int(min_height_cells)
        self.min_area = int(min_area_cells)
        self.allow_shrink = bool(allow_shrink)
        self.max_shrink = int(max_shrink_per_update)
    
    def update_from_grid(self,
                        grid_map,
                        ensure_contains: Optional[Tuple[int, int]] = None) -> Optional[Tuple[int, int, int, int]]:
        """
        扫描当前网格，计算"已知障碍物"的包围盒 → 外扩margin → 与历史ROI融合 → 返回矩形
        
        Args:
            grid_map: numpy数组，地图状态
            ensure_contains: 确保包含的点 (grid_x, grid_y)，通常是机器人当前位置
            
        Returns:
            矩形 (x0, y0, x1, y1) 或 None
        """
        H, W = grid_map.shape
        
        # 1) 收集已知障碍物的极值
        x0, y0 = W, H
        x1, y1 = -1, -1
        obstacle_count = 0
        
        for y in range(H):
            for x in range(W):
                # 只考虑障碍物来确定迷宫边界
                if grid_map[y, x] == config.MAP_STATE_OCCUPIED:
                    obstacle_count += 1
                    if x < x0:
                        x0 = x
                    if x > x1:
                        x1 = x
                    if y < y0:
                        y0 = y
                    if y > y1:
                        y1 = y
        
        if obstacle_count == 0:
            # 还没观测到障碍物：维持旧值或None
            return self.rect
        
        # 初始包围盒
        if x1 < x0 or y1 < y0:
            return self.rect
        
        # 2) 最小尺寸/面积过滤
        w = x1 - x0 + 1
        h = y1 - y0 + 1
        if w < self.min_w or h < self.min_h or (w * h) < self.min_area:
            # 尺寸太小：可能观测刚开始；不要刷新ROI
            return self.rect
        
        # 3) 外扩margin并裁剪到地图范围
        x0m = max(0, x0 - self.margin)
        y0m = max(0, y0 - self.margin)
        x1m = min(W - 1, x1 + self.margin)
        y1m = min(H - 1, y1 + self.margin)
        
        # 确保包含指定点（通常是机器人位置）
        if ensure_contains is not None:
            cx, cy = ensure_contains
            x0m = min(x0m, max(0, cx))
            y0m = min(y0m, max(0, cy))
            x1m = max(x1m, min(W - 1, cx))
            y1m = max(y1m, min(H - 1, cy))
        
        candidate = (x0m, y0m, x1m, y1m)
        
        # 4) 与历史ROI融合 —— 缺省"只增不减"
        if self.rect is None:
            self.rect = candidate
            return self.rect
        
        cx0, cy0, cx1, cy1 = candidate
        px0, py0, px1, py1 = self.rect
        
        # 扩张：取并集（更稳）
        nx0 = min(px0, cx0)
        ny0 = min(py0, cy0)
        nx1 = max(px1, cx1)
        ny1 = max(py1, cy1)
        
        if self.allow_shrink:
            # 允许缓慢收缩：每次最多内缩max_shrink格子
            nx0 = self._soft_shrink(px0, cx0, inward=True)
            ny0 = self._soft_shrink(py0, cy0, inward=True)
            nx1 = self._soft_shrink(px1, cx1, inward=False)
            ny1 = self._soft_shrink(py1, cy1, inward=False)
            
            # 确保不会把盒子变得过小
            if (nx1 - nx0 + 1) < self.min_w or (ny1 - ny0 + 1) < self.min_h:
                nx0, ny0, nx1, ny1 = px0, py0, px1, py1  # 放弃收缩
        
        self.rect = (nx0, ny0, nx1, ny1)
        return self.rect
    
    def _soft_shrink(self, prev_edge: int, cand_edge: int, inward: bool) -> int:
        """
        缓慢收缩边界
        
        Args:
            prev_edge: 旧ROI的某一条边
            cand_edge: 新候选盒子的对应边
            inward: True表示左/上边，False表示右/下边
        """
        if inward:
            # 左/上边：cand_edge > prev_edge 表示向内收缩
            if cand_edge <= prev_edge:
                return cand_edge  # 扩张或等同
            else:
                return min(prev_edge + self.max_shrink, cand_edge)
        else:
            # 右/下边：cand_edge < prev_edge 表示向内收缩
            if cand_edge >= prev_edge:
                return cand_edge  # 扩张或等同
            else:
                return max(prev_edge - self.max_shrink, cand_edge)
    
    def is_inside(self, grid_x: int, grid_y: int) -> bool:
        """检查栅格坐标是否落在有效矩形里"""
        if not self.rect:
            return True
        x0, y0, x1, y1 = self.rect
        return (x0 <= grid_x <= x1) and (y0 <= grid_y <= y1)
    
    def get_roi(self) -> Optional[Tuple[int, int, int, int]]:
        """返回当前ROI（栅格坐标系）"""
        return self.rect
    
    def reset(self):
        """重置ROI"""
        self.rect = None

