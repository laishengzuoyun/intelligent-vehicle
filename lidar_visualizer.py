"""
雷达数据可视化模块
使用极坐标图显示2D激光雷达扫描数据
"""

import numpy as np
import matplotlib
# 设置matplotlib使用更快的后端
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import config

# 设置matplotlib中文字体和性能优化
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False
# 性能优化设置
matplotlib.rcParams['figure.max_open_warning'] = 0
matplotlib.rcParams['path.simplify'] = True
matplotlib.rcParams['path.simplify_threshold'] = 0.1
matplotlib.rcParams['agg.path.chunksize'] = 10000


class LidarVisualizer(FigureCanvas):
    """雷达数据可视化组件"""
    
    def __init__(self, parent=None):
        # 创建Figure - 大幅降低DPI提升性能
        self.fig = Figure(figsize=(6, 4), dpi=30)  # 从60降到30，减少4倍渲染负担
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        super().__init__(self.fig)
        self.setParent(parent)
        
        # 雷达数据
        self.lidar_data = []  # [(angle, distance), ...]
        
        # 性能优化：缓存渲染对象
        self._scatter_plot = None  # 雷达点散点图对象
        self._last_update_time = 0  # 上次更新时间
        
        # 设置样式
        self.fig.patch.set_facecolor('#0a0a28')
        self.ax.set_facecolor('#0a0a28')
        
        # 初始化显示
        self.update_display()
    
    def update_data(self, lidar_data):
        """更新雷达数据 - 平衡版本，保持雷达显示速度
        
        Args:
            lidar_data: 雷达数据列表 [(angle_deg, distance_mm), ...]
        """
        import time
        
        # 雷达更新间隔：从300ms增加到500ms，进一步减少更新频率
        current_time = time.time() * 1000
        if current_time - self._last_update_time < 500:
            return
        
        # 适当增加数据点数量，从50个增加到100个，保持显示效果
        max_points = 100  # 显示100个点，平衡性能和效果
        if len(lidar_data) > max_points:
            # 均匀采样
            step = len(lidar_data) // max_points
            self.lidar_data = lidar_data[::step]
        else:
            self.lidar_data = lidar_data
        
        self._last_update_time = current_time
        self.update_display()
    
    def update_display(self):
        """更新显示 - 优化版本，使用增量更新"""
        # 移除旧的散点图
        if self._scatter_plot:
            self._scatter_plot.remove()
            self._scatter_plot = None
        
        if self.lidar_data:
            # 提取角度和距离
            angles = []
            distances = []
            
            for angle, distance in self.lidar_data:
                # 转换角度为弧度
                angle_rad = np.radians(angle)
                angles.append(angle_rad)
                distances.append(distance)
            
            # 绘制雷达点
            self._scatter_plot = self.ax.scatter(angles, distances,
                              color=self._rgb_to_normalized(config.COLOR_LIDAR),
                              s=15, alpha=0.8, marker='o')  # 减小点的大小
        
        # 只在初始化时设置样式
        if not hasattr(self, '_style_initialized'):
            self._setup_style()
            self._style_initialized = True

        # 刷新画布 - 使用draw_idle()避免递归
        self.draw_idle()
    
    def _setup_style(self):
        """设置极坐标图样式"""
        # 设置极坐标图样式
        self.ax.set_theta_zero_location('E')  # 0度在东方（右侧）
        self.ax.set_theta_direction(1)  # 逆时针
        
        # 设置径向范围
        self.ax.set_ylim(0, config.LIDAR_MAX_RANGE)
        
        # 设置网格
        self.ax.grid(True, alpha=0.3, color='white', linestyle='--', linewidth=0.5)
        
        # 设置标题
        self.ax.set_title('激光雷达扫描', color='white', fontsize=12, 
                         fontweight='bold', pad=20)
        
        # 设置刻度颜色
        self.ax.tick_params(colors='white')
        
        # 设置径向标签
        self.ax.set_ylabel('距离 (mm)', color='white', labelpad=30)
    
    @staticmethod
    def _rgb_to_normalized(rgb):
        """RGB (0-255) 转换为归一化 (0-1)"""
        return tuple(c / 255.0 for c in rgb)

