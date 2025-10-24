"""
独立雷达数据查看器
圆形显示雷达扫描数据，显示被扫描的面积
"""

import sys
import numpy as np
import matplotlib
# 设置matplotlib使用更快的后端
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Wedge
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QGroupBox, QSlider, 
                             QSpinBox, QGridLayout, QCheckBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

import config


class RadarViewer(QMainWindow):
    """独立雷达数据查看器"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 雷达数据
        self.lidar_data = []  # [(angle, distance), ...]
        self.scanned_area = 0.0  # 扫描面积（平方米）
        self.max_range = config.LIDAR_MAX_RANGE  # 最大扫描距离
        
        # 显示设置
        self.show_scanned_area = True
        self.show_distance_rings = True
        self.show_angle_lines = True
        self.point_size = 15
        # 优化：降低更新频率，从100ms增加到200ms
        self.update_interval = 200  # 更新间隔（毫秒）
        
        # 初始化UI
        self.init_ui()
        
        # 启动定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(self.update_interval)
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('雷达数据查看器 - 迷宫导航机器人')
        self.setGeometry(300, 300, 800, 600)
        
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, stretch=1)
        
        # 右侧雷达显示
        right_panel = self.create_radar_panel()
        main_layout.addWidget(right_panel, stretch=3)
        
    def create_control_panel(self):
        """创建左侧控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 1. 显示设置组
        display_group = QGroupBox("显示设置")
        display_layout = QVBoxLayout()
        
        # 显示选项
        self.check_scanned_area = QCheckBox("显示扫描面积")
        self.check_scanned_area.setChecked(True)
        self.check_scanned_area.toggled.connect(self.toggle_scanned_area)
        display_layout.addWidget(self.check_scanned_area)
        
        self.check_distance_rings = QCheckBox("显示距离环")
        self.check_distance_rings.setChecked(True)
        self.check_distance_rings.toggled.connect(self.toggle_distance_rings)
        display_layout.addWidget(self.check_distance_rings)
        
        self.check_angle_lines = QCheckBox("显示角度线")
        self.check_angle_lines.setChecked(True)
        self.check_angle_lines.toggled.connect(self.toggle_angle_lines)
        display_layout.addWidget(self.check_angle_lines)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # 2. 参数设置组
        param_group = QGroupBox("参数设置")
        param_layout = QVBoxLayout()
        
        # 点大小设置
        point_layout = QHBoxLayout()
        point_layout.addWidget(QLabel("点大小:"))
        self.slider_point_size = QSlider(Qt.Horizontal)
        self.slider_point_size.setRange(5, 50)
        self.slider_point_size.setValue(self.point_size)
        self.slider_point_size.valueChanged.connect(self.update_point_size)
        point_layout.addWidget(self.slider_point_size)
        self.lbl_point_size = QLabel(str(self.point_size))
        point_layout.addWidget(self.lbl_point_size)
        param_layout.addLayout(point_layout)
        
        # 最大距离设置
        range_layout = QHBoxLayout()
        range_layout.addWidget(QLabel("最大距离:"))
        self.spin_max_range = QSpinBox()
        self.spin_max_range.setRange(1000, 10000)
        self.spin_max_range.setValue(self.max_range)
        self.spin_max_range.setSingleStep(500)
        self.spin_max_range.valueChanged.connect(self.update_max_range)
        range_layout.addWidget(self.spin_max_range)
        range_layout.addWidget(QLabel("mm"))
        param_layout.addLayout(range_layout)
        
        # 更新间隔设置
        interval_layout = QHBoxLayout()
        interval_layout.addWidget(QLabel("更新间隔:"))
        self.spin_interval = QSpinBox()
        self.spin_interval.setRange(50, 1000)
        self.spin_interval.setValue(self.update_interval)
        self.spin_interval.setSingleStep(50)
        self.spin_interval.valueChanged.connect(self.update_interval_setting)
        interval_layout.addWidget(self.spin_interval)
        interval_layout.addWidget(QLabel("ms"))
        param_layout.addLayout(interval_layout)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 3. 统计信息组
        stats_group = QGroupBox("统计信息")
        stats_layout = QVBoxLayout()
        
        self.lbl_point_count = QLabel("扫描点数: 0")
        self.lbl_point_count.setFont(QFont("Courier", 9))
        stats_layout.addWidget(self.lbl_point_count)
        
        self.lbl_scanned_area = QLabel("扫描面积: 0.00 m²")
        self.lbl_scanned_area.setFont(QFont("Courier", 9))
        stats_layout.addWidget(self.lbl_scanned_area)
        
        self.lbl_coverage = QLabel("覆盖率: 0.0%")
        self.lbl_coverage.setFont(QFont("Courier", 9))
        stats_layout.addWidget(self.lbl_coverage)
        
        self.lbl_avg_distance = QLabel("平均距离: 0 mm")
        self.lbl_avg_distance.setFont(QFont("Courier", 9))
        stats_layout.addWidget(self.lbl_avg_distance)
        
        self.lbl_min_distance = QLabel("最小距离: 0 mm")
        self.lbl_min_distance.setFont(QFont("Courier", 9))
        stats_layout.addWidget(self.lbl_min_distance)
        
        self.lbl_max_distance = QLabel("最大距离: 0 mm")
        self.lbl_max_distance.setFont(QFont("Courier", 9))
        stats_layout.addWidget(self.lbl_max_distance)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        # 4. 控制按钮组
        control_group = QGroupBox("控制")
        control_layout = QVBoxLayout()
        
        self.btn_clear_data = QPushButton("清空数据")
        self.btn_clear_data.clicked.connect(self.clear_data)
        control_layout.addWidget(self.btn_clear_data)
        
        self.btn_pause = QPushButton("暂停更新")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self.toggle_pause)
        control_layout.addWidget(self.btn_pause)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        layout.addStretch()
        return panel
    
    def create_radar_panel(self):
        """创建右侧雷达面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 雷达可视化
        self.radar_canvas = RadarCanvas()
        layout.addWidget(self.radar_canvas)
        
        return panel
    
    def update_lidar_data(self, lidar_data):
        """更新雷达数据"""
        self.lidar_data = lidar_data
        self.calculate_scanned_area()
        self.update_statistics()
    
    def calculate_scanned_area(self):
        """计算扫描面积"""
        if not self.lidar_data:
            self.scanned_area = 0.0
            return
        
        # 使用扇形面积公式计算扫描面积
        # 假设雷达扫描360度
        total_angle = 360.0  # 度
        
        # 计算有效扫描点（距离小于最大距离的点）
        valid_points = [d for _, d in self.lidar_data if d < self.max_range]
        
        if not valid_points:
            self.scanned_area = 0.0
            return
        
        # 计算平均距离
        avg_distance = np.mean(valid_points)
        
        # 扇形面积 = (角度/360) * π * r²
        # 这里简化为圆形面积的一部分
        angle_rad = np.radians(total_angle)
        self.scanned_area = (angle_rad / (2 * np.pi)) * np.pi * (avg_distance / 1000) ** 2  # 转换为平方米
    
    def update_statistics(self):
        """更新统计信息"""
        if not self.lidar_data:
            self.lbl_point_count.setText("扫描点数: 0")
            self.lbl_scanned_area.setText("扫描面积: 0.00 m²")
            self.lbl_coverage.setText("覆盖率: 0.0%")
            self.lbl_avg_distance.setText("平均距离: 0 mm")
            self.lbl_min_distance.setText("最小距离: 0 mm")
            self.lbl_max_distance.setText("最大距离: 0 mm")
            return
        
        point_count = len(self.lidar_data)
        self.lbl_point_count.setText(f"扫描点数: {point_count}")
        
        # 扫描面积
        self.lbl_scanned_area.setText(f"扫描面积: {self.scanned_area:.2f} m²")
        
        # 覆盖率（相对于最大可能扫描面积）
        max_possible_area = np.pi * (self.max_range / 1000) ** 2  # 平方米
        coverage = (self.scanned_area / max_possible_area) * 100 if max_possible_area > 0 else 0
        self.lbl_coverage.setText(f"覆盖率: {coverage:.1f}%")
        
        # 距离统计
        distances = [d for _, d in self.lidar_data]
        if distances:
            avg_dist = np.mean(distances)
            min_dist = np.min(distances)
            max_dist = np.max(distances)
            
            self.lbl_avg_distance.setText(f"平均距离: {avg_dist:.0f} mm")
            self.lbl_min_distance.setText(f"最小距离: {min_dist:.0f} mm")
            self.lbl_max_distance.setText(f"最大距离: {max_dist:.0f} mm")
    
    def update_display(self):
        """更新显示"""
        if not self.btn_pause.isChecked():
            self.radar_canvas.update_data(
                self.lidar_data,
                show_scanned_area=self.show_scanned_area,
                show_distance_rings=self.show_distance_rings,
                show_angle_lines=self.show_angle_lines,
                point_size=self.point_size,
                max_range=self.max_range,
                scanned_area=self.scanned_area
            )
    
    def toggle_scanned_area(self, checked):
        """切换扫描面积显示"""
        self.show_scanned_area = checked
    
    def toggle_distance_rings(self, checked):
        """切换距离环显示"""
        self.show_distance_rings = checked
    
    def toggle_angle_lines(self, checked):
        """切换角度线显示"""
        self.show_angle_lines = checked
    
    def update_point_size(self, value):
        """更新点大小"""
        self.point_size = value
        self.lbl_point_size.setText(str(value))
    
    def update_max_range(self, value):
        """更新最大距离"""
        self.max_range = value
        self.calculate_scanned_area()
        self.update_statistics()
    
    def update_interval_setting(self, value):
        """更新更新间隔"""
        self.update_interval = value
        self.timer.setInterval(value)
    
    def clear_data(self):
        """清空数据"""
        self.lidar_data = []
        self.scanned_area = 0.0
        self.update_statistics()
        self.radar_canvas.clear_data()
    
    def toggle_pause(self, checked):
        """切换暂停状态"""
        if checked:
            self.btn_pause.setText("继续更新")
        else:
            self.btn_pause.setText("暂停更新")


class RadarCanvas(FigureCanvas):
    """雷达数据画布"""
    
    def __init__(self, parent=None):
        # 创建Figure
        self.fig = Figure(figsize=(8, 6), dpi=50)
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        super().__init__(self.fig)
        self.setParent(parent)
        
        # 雷达数据
        self.lidar_data = []
        
        # 显示设置
        self.show_scanned_area = True
        self.show_distance_rings = True
        self.show_angle_lines = True
        self.point_size = 15
        self.max_range = config.LIDAR_MAX_RANGE
        self.scanned_area = 0.0
        
        # 性能优化：缓存渲染对象
        self._scatter_plot = None
        self._scanned_area_patch = None
        self._distance_rings = []
        self._angle_lines = []
        
        # 设置样式
        self.fig.patch.set_facecolor('#0a0a28')
        self.ax.set_facecolor('#0a0a28')
        
        # 初始化显示
        self.setup_style()
        self.update_display()
    
    def setup_style(self):
        """设置极坐标图样式"""
        # 设置极坐标图样式
        self.ax.set_theta_zero_location('E')  # 0度在东方（右侧）
        self.ax.set_theta_direction(1)  # 逆时针
        
        # 设置径向范围
        self.ax.set_ylim(0, self.max_range)
        
        # 设置网格
        self.ax.grid(True, alpha=0.3, color='white', linestyle='--', linewidth=0.5)
        
        # 设置标题
        self.ax.set_title('激光雷达扫描数据', color='white', fontsize=14, 
                         fontweight='bold', pad=20)
        
        # 设置刻度颜色
        self.ax.tick_params(colors='white')
        
        # 设置径向标签
        self.ax.set_ylabel('距离 (mm)', color='white', labelpad=30)
    
    def update_data(self, lidar_data, show_scanned_area=True, show_distance_rings=True, 
                   show_angle_lines=True, point_size=15, max_range=4000, scanned_area=0.0):
        """更新雷达数据"""
        self.lidar_data = lidar_data
        self.show_scanned_area = show_scanned_area
        self.show_distance_rings = show_distance_rings
        self.show_angle_lines = show_angle_lines
        self.point_size = point_size
        self.max_range = max_range
        self.scanned_area = scanned_area
        
        # 更新径向范围
        self.ax.set_ylim(0, max_range)
        
        self.update_display()
    
    def update_display(self):
        """更新显示"""
        # 清除旧的动态元素
        self.clear_dynamic_elements()
        
        # 绘制距离环
        if self.show_distance_rings:
            self.draw_distance_rings()
        
        # 绘制角度线
        if self.show_angle_lines:
            self.draw_angle_lines()
        
        # 绘制扫描面积
        if self.show_scanned_area and self.scanned_area > 0:
            self.draw_scanned_area()
        
        # 绘制雷达点
        if self.lidar_data:
            self.draw_lidar_points()
        
        # 刷新画布
        self.draw_idle()
    
    def clear_dynamic_elements(self):
        """清除动态元素"""
        # 移除旧的散点图
        if self._scatter_plot:
            self._scatter_plot.remove()
            self._scatter_plot = None
        
        # 移除扫描面积
        if self._scanned_area_patch:
            self._scanned_area_patch.remove()
            self._scanned_area_patch = None
        
        # 移除距离环
        for ring in self._distance_rings:
            ring.remove()
        self._distance_rings.clear()
        
        # 移除角度线
        for line in self._angle_lines:
            line.remove()
        self._angle_lines.clear()
    
    def draw_distance_rings(self):
        """绘制距离环"""
        # 绘制几个距离环
        ring_distances = [1000, 2000, 3000, 4000]
        for dist in ring_distances:
            if dist <= self.max_range:
                circle = Circle((0, 0), dist, fill=False, 
                              color='cyan', alpha=0.3, linewidth=1)
                self.ax.add_patch(circle)
                self._distance_rings.append(circle)
    
    def draw_angle_lines(self):
        """绘制角度线"""
        # 绘制主要角度线（每30度）
        for angle in range(0, 360, 30):
            angle_rad = np.radians(angle)
            line = self.ax.plot([angle_rad, angle_rad], [0, self.max_range], 
                               color='cyan', alpha=0.2, linewidth=0.5)[0]
            self._angle_lines.append(line)
    
    def draw_scanned_area(self):
        """绘制扫描面积"""
        if not self.lidar_data:
            return
        
        # 计算有效扫描点
        valid_points = [(a, d) for a, d in self.lidar_data if d < self.max_range]
        
        if not valid_points:
            return
        
        # 计算平均距离
        avg_distance = np.mean([d for _, d in valid_points])
        
        # 绘制扫描面积扇形
        wedge = Wedge((0, 0), avg_distance, 0, 360, 
                     facecolor='green', alpha=0.1, edgecolor='green', linewidth=2)
        self.ax.add_patch(wedge)
        self._scanned_area_patch = wedge
    
    def draw_lidar_points(self):
        """绘制雷达点"""
        if not self.lidar_data:
            return
        
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
                          s=self.point_size, alpha=0.8, marker='o')
    
    def clear_data(self):
        """清空数据"""
        self.lidar_data = []
        self.scanned_area = 0.0
        self.clear_dynamic_elements()
        self.draw_idle()
    
    @staticmethod
    def _rgb_to_normalized(rgb):
        """RGB (0-255) 转换为归一化 (0-1)"""
        return tuple(c / 255.0 for c in rgb)


if __name__ == '__main__':
    from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    # 设置中文字体
    font = QFont()
    font.setFamily("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)
    
    window = RadarViewer()
    window.show()
    
    # 模拟雷达数据
    import random
    def simulate_lidar_data():
        data = []
        for angle in range(0, 360, 5):
            distance = random.randint(500, 3500)
            data.append((angle, distance))
        window.update_lidar_data(data)
    
    # 定时模拟数据
    sim_timer = QTimer()
    sim_timer.timeout.connect(simulate_lidar_data)
    sim_timer.start(1000)
    
    sys.exit(app.exec_())

