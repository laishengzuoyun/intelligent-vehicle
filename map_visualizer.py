"""
地图可视化模块
使用Matplotlib在PyQt5中显示占据栅格地图
"""

import numpy as np
import matplotlib
# 设置matplotlib使用更快的后端
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import pyqtSignal
import math
import config

# 设置matplotlib中文字体和性能优化
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False
# 性能优化设置
matplotlib.rcParams['figure.max_open_warning'] = 0
matplotlib.rcParams['path.simplify'] = True
matplotlib.rcParams['path.simplify_threshold'] = 0.1
matplotlib.rcParams['agg.path.chunksize'] = 10000


class MapVisualizer(FigureCanvas):
    """地图可视化组件"""
    
    # 信号：点击地图时发出 (world_x, world_y)
    point_clicked = pyqtSignal(float, float)
    
    def __init__(self, map_manager, parent=None):
        self.map_manager = map_manager

        # 创建Figure - 大幅降低DPI提升性能
        self.fig = Figure(figsize=(8, 6), dpi=30)  # 从60降到30，减少4倍渲染负担
        self.ax = self.fig.add_subplot(111)

        super().__init__(self.fig)
        self.setParent(parent)

        # 机器人状态
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0

        # 轨迹和路径
        self.trajectory = []
        self.path = None

        # 新增：前沿探索可视化
        self.all_frontiers = []  # 所有前沿点
        self.best_frontiers = []  # 最佳前沿点
        self.maze_rect = None  # ROI区域 (x0, y0, x1, y1)
        self.exit_position = None  # 出口位置 (grid_x, grid_y)

        # 性能优化：缓存渲染对象
        self._grid_image = None  # 缓存栅格地图图像
        self._last_grid_hash = None  # 栅格地图哈希值
        self._robot_circle = None  # 机器人圆形对象
        self._robot_arrow = None  # 机器人箭头对象
        self._trajectory_line = None  # 轨迹线对象
        self._path_line = None  # 路径线对象
        self._frontier_scatter = None  # 前沿点散点图对象
        self._best_frontier_scatter = None  # 最佳前沿点散点图对象

        # 设置样式
        self.fig.patch.set_facecolor('#808080')  # 灰色背景
        self.ax.set_facecolor('#808080')

        # 连接鼠标点击事件
        self.mpl_connect('button_press_event', self.on_click)

        # 初始化显示
        self.update()
    
    def set_robot_pose(self, x, y, theta):
        """设置机器人位姿"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        # 删除频繁的DEBUG输出，提升性能
    
    def set_trajectory(self, trajectory):
        """设置轨迹"""
        self.trajectory = trajectory
    
    def set_path(self, path):
        """设置规划路径"""
        self.path = path

    def set_frontiers(self, all_frontiers, best_frontiers):
        """设置前沿点"""
        self.all_frontiers = all_frontiers
        self.best_frontiers = best_frontiers

    def set_maze_rect(self, maze_rect):
        """设置迷宫ROI区域"""
        self.maze_rect = maze_rect

    def reset_map_cache(self):
        """重置地图缓存，强制下次更新时重绘"""
        self._last_grid_hash = None
        self._grid_image = None
        self._last_map_update_time = 0
        print("地图缓存已重置，下次更新将强制重绘")
    
    def set_exit_position(self, exit_position):
        """设置出口位置"""
        self.exit_position = exit_position
    
    def update(self, force_full_redraw=False):
        """更新显示 - 修复哈希缓存问题，确保建图及时显示"""
        import hashlib
        import time
        
        # 限制更新频率，但保持建图显示速度
        current_time = time.time() * 1000
        if not hasattr(self, '_last_map_update_time'):
            self._last_map_update_time = 0
        
        # 地图更新间隔：50ms，大幅提高建图显示速度
        if current_time - self._last_map_update_time < 50 and not force_full_redraw:
            return
        
        self._last_map_update_time = current_time
        
        # 修复哈希检测问题：每次更新都重新计算哈希
        grid_hash = hashlib.md5(self.map_manager.grid_map.tobytes()).hexdigest()
        grid_changed = (self._last_grid_hash != grid_hash)
        
        # 强制更新条件：地图变化、强制重绘、或首次显示
        should_redraw = force_full_redraw or grid_changed or self._grid_image is None
        
        # 调试信息：显示哈希变化情况
        if grid_changed and self._last_grid_hash is not None:
            pass  # 删除调试输出，减少CPU占用
        
        if should_redraw:
            # 完全重绘
            self.ax.clear()
            self._grid_image = None
            self._last_grid_hash = grid_hash
            
            # 绘制栅格地图
            self.draw_grid_map()
            
            # 绘制墙壁（从JSON加载的）
            self.draw_walls()
            
            # 绘制ROI区域
            self.draw_maze_rect()
            
            # 绘制出口位置
            self.draw_exit()
            
            # 设置坐标轴
            self.ax.set_xlim(0, self.map_manager.map_width)
            self.ax.set_ylim(0, self.map_manager.map_height)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3, color='white', linestyle='--', linewidth=0.5)
            self.ax.set_xlabel('X (mm)', color='white')
            self.ax.set_ylabel('Y (mm)', color='white')
            self.ax.set_title('迷宫地图', color='white', fontsize=14, fontweight='bold')
            self.ax.tick_params(colors='white')
            
            # 绘制动态元素（小车、轨迹、路径、前沿点）
            self.draw_trajectory()
            self.draw_path()
            self.draw_frontiers()
            self.draw_robot()
        else:
            # 增量更新：只更新动态元素
            self.update_dynamic_elements()
        
        # 刷新画布 - 使用draw_idle()避免递归
        self.draw_idle()
    
    def update_dynamic_elements(self):
        """更新动态元素（机器人、轨迹、路径、前沿点）"""
        # 移除旧的动态元素
        if self._robot_circle:
            self._robot_circle.remove()
            self._robot_circle = None
        if self._robot_arrow:
            self._robot_arrow.remove()
            self._robot_arrow = None
        if self._trajectory_line:
            self._trajectory_line.remove()
            self._trajectory_line = None
        if self._path_line:
            self._path_line.remove()
            self._path_line = None
        if self._frontier_scatter:
            self._frontier_scatter.remove()
            self._frontier_scatter = None
        if self._best_frontier_scatter:
            self._best_frontier_scatter.remove()
            self._best_frontier_scatter = None
        
        # 重新绘制动态元素
        self.draw_trajectory()
        self.draw_path()
        self.draw_frontiers()
        self.draw_robot()
    
    def draw_grid_map(self):
        """绘制栅格地图"""
        grid = self.map_manager.grid_map
        
        # 创建颜色映射的图像
        img = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        
        # 根据地图状态设置颜色
        for state, color in [
            (config.MAP_STATE_UNKNOWN, config.COLOR_BACKGROUND),    # 灰色背景
            (config.MAP_STATE_FREE, config.COLOR_FREE),            # 白色已探索区域
            (config.MAP_STATE_OCCUPIED, config.COLOR_WALL),         # 深灰色障碍物
            (config.MAP_STATE_ENCLOSED, (100, 100, 100))            # 中灰色封闭区域
        ]:
            mask = (grid == state)
            img[mask] = color
        
        # 显示图像（注意Y轴翻转）
        extent = [0, self.map_manager.map_width, 0, self.map_manager.map_height]
        self.ax.imshow(img, origin='lower', extent=extent, alpha=0.8)
    
    def draw_walls(self):
        """绘制墙壁"""
        for wall in self.map_manager.walls:
            (x1, y1), (x2, y2) = wall
            self.ax.plot([x1, x2], [y1, y2], 
                        color=self._rgb_to_normalized(config.COLOR_WALL),
                        linewidth=3, alpha=0.9)
    
    def draw_trajectory(self):
        """绘制机器人轨迹"""
        if len(self.trajectory) > 1:
            traj_array = np.array(self.trajectory)
            self._trajectory_line, = self.ax.plot(traj_array[:, 0], traj_array[:, 1],
                        color=self._rgb_to_normalized(config.COLOR_TRAJECTORY),
                        linewidth=2, alpha=0.6, linestyle='-')
    
    def draw_path(self):
        """绘制规划路径"""
        if self.path and len(self.path) > 1:
            # 转换栅格坐标到世界坐标
            path_world = []
            for gx, gy in self.path:
                wx, wy = self.map_manager.grid_to_world(gx, gy)
                path_world.append((wx, wy))
            
            path_array = np.array(path_world)
            self._path_line, = self.ax.plot(path_array[:, 0], path_array[:, 1],
                        color=self._rgb_to_normalized(config.COLOR_PATH),
                        linewidth=3, alpha=0.8, linestyle='--',
                        marker='o', markersize=4)
    
    def draw_maze_rect(self):
        """绘制迷宫ROI区域"""
        if self.maze_rect is None:
            return

        x0, y0, x1, y1 = self.maze_rect
        # 转换为世界坐标
        wx0 = x0 * config.CELL_SIZE
        wy0 = y0 * config.CELL_SIZE
        wx1 = x1 * config.CELL_SIZE
        wy1 = y1 * config.CELL_SIZE

        # 绘制ROI矩形
        from matplotlib.patches import Rectangle
        rect = Rectangle((wx0, wy0), wx1 - wx0, wy1 - wy0,
                        linewidth=2, edgecolor='cyan', facecolor='none',
                        linestyle='--', alpha=0.5)
        self.ax.add_patch(rect)

        # 添加标签
        self.ax.text(wx0 + 50, wy1 - 50, 'ROI',
                    color='cyan', fontsize=10, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.7))

    def draw_frontiers(self):
        """绘制前沿点（优化版本，限制数量避免卡顿）"""
        # 绘制所有前沿点（灰色小点）- 限制最多50个避免卡顿
        if self.all_frontiers:
            # 采样：如果前沿点太多，只绘制部分
            max_display = 50  # 进一步减少显示数量
            if len(self.all_frontiers) > max_display:
                import random
                display_frontiers = random.sample(self.all_frontiers, max_display)
            else:
                display_frontiers = self.all_frontiers

            frontier_world = [self.map_manager.grid_to_world(gx, gy)
                            for gx, gy in display_frontiers]
            if frontier_world:
                fx, fy = zip(*frontier_world)
                self._frontier_scatter = self.ax.scatter(fx, fy, c='gray', s=6, alpha=0.3,
                              marker='o', label=f'前沿点({len(self.all_frontiers)})')

        # 绘制最佳前沿点（黄色大点）
        if self.best_frontiers:
            best_world = [self.map_manager.grid_to_world(gx, gy)
                         for gx, gy in self.best_frontiers]
            if best_world:
                bx, by = zip(*best_world)
                self._best_frontier_scatter = self.ax.scatter(bx, by, c='yellow', s=80, alpha=0.8,
                              marker='*', edgecolors='orange', linewidths=2,
                              label=f'最佳前沿({len(self.best_frontiers)})', zorder=10)

                # 标注第一个最佳前沿点
                if len(best_world) > 0:
                    self.ax.text(bx[0], by[0] + 100, '目标',
                               color='yellow', fontsize=9, fontweight='bold',
                               ha='center',
                               bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.7))

    def draw_exit(self):
        """绘制出口位置"""
        if self.exit_position is None:
            return

        gx, gy = self.exit_position
        wx, wy = self.map_manager.grid_to_world(gx, gy)

        # 绘制出口标记（绿色五角星）
        self.ax.scatter([wx], [wy], c='lime', s=300, alpha=0.9,
                       marker='*', edgecolors='green', linewidths=3,
                       label='出口', zorder=15)

        # 添加文字标签
        self.ax.text(wx, wy + 150, 'EXIT',
                    color='lime', fontsize=12, fontweight='bold',
                    ha='center',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='green', alpha=0.7))

    def draw_robot(self):
        """绘制机器人"""
        # 删除频繁的DEBUG输出，提升性能
        # 机器人本体（圆形）
        self._robot_circle = plt.Circle((self.robot_x, self.robot_y),
                           config.ROBOT_RADIUS,
                           color=self._rgb_to_normalized(config.COLOR_ROBOT),
                           alpha=0.8)
        self.ax.add_patch(self._robot_circle)

        # 机器人朝向（箭头）
        arrow_length = config.ROBOT_RADIUS * 2
        dx = arrow_length * math.cos(math.radians(self.robot_theta))
        dy = arrow_length * math.sin(math.radians(self.robot_theta))

        self._robot_arrow = self.ax.arrow(self.robot_x, self.robot_y, dx, dy,
                     head_width=config.ROBOT_RADIUS * 0.8,
                     head_length=config.ROBOT_RADIUS * 0.8,
                     fc=self._rgb_to_normalized((255, 255, 255)),
                     ec=self._rgb_to_normalized((255, 255, 255)),
                     linewidth=2)

        # 机器人位置标注
        self.ax.text(self.robot_x, self.robot_y - config.ROBOT_RADIUS * 3,
                    f'({self.robot_x:.0f}, {self.robot_y:.0f})',
                    color='white', fontsize=8, ha='center',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.5))
    
    def on_click(self, event):
        """鼠标点击事件"""
        if event.inaxes == self.ax and event.button == 1:  # 左键点击
            # 获取点击的世界坐标
            world_x = event.xdata
            world_y = event.ydata
            
            if world_x is not None and world_y is not None:
                self.point_clicked.emit(world_x, world_y)
    
    @staticmethod
    def _rgb_to_normalized(rgb):
        """RGB (0-255) 转换为归一化 (0-1)"""
        return tuple(c / 255.0 for c in rgb)

