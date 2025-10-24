"""
独立建图窗口
显示地图、轨迹、路径和前沿点等建图相关信息
"""

import sys
import os
import math
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QGroupBox, QFileDialog,
                             QMessageBox, QSpinBox, QGridLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont

import config
from map_manager import OccupancyGridMap
from map_visualizer import MapVisualizer
from path_planner import PathPlanner
from frontier_explorer import OptimizedFrontierExplorer
from goal_checker import GoalReachabilityChecker
from map_rectifier import MapRectifier


class MappingWindow(QMainWindow):
    """独立建图窗口"""
    
    # 信号：用于与主窗口通信
    map_clicked = pyqtSignal(float, float)  # 地图点击事件
    exploration_started = pyqtSignal()  # 探索开始
    exploration_stopped = pyqtSignal()  # 探索停止
    
    def __init__(self, map_manager=None, path_planner=None, parent=None):
        super().__init__(parent)
        
        # 接收外部组件或创建新实例
        if map_manager is not None:
            self.map_manager = map_manager
        else:
            self.map_manager = OccupancyGridMap(
                config.WORLD_WIDTH,
                config.WORLD_HEIGHT,
                config.CELL_SIZE
            )
        
        if path_planner is not None:
            self.path_planner = path_planner
        else:
            self.path_planner = PathPlanner(self.map_manager)
        
        # 机器人状态
        self.robot_x = config.ROBOT_INIT_X
        self.robot_y = config.ROBOT_INIT_Y
        self.robot_theta = config.ROBOT_INIT_THETA
        self.trajectory = []
        
        # 路径和探索状态
        self.current_path = None
        self.goal_position = None
        self.exploration_mode = False
        
        # 探索相关组件
        self.frontier_explorer = OptimizedFrontierExplorer(
            obs_clearance_cells=2,
            min_distance_cells=3,
            max_distance_cells=200,
            goal_bias_weight=0.6
        )
        self.goal_checker = GoalReachabilityChecker(
            check_radius_cells=5,
            free_ratio_threshold=0.7
        )
        self.map_rectifier = MapRectifier(
            margin_cells=20,
            allow_shrink=False
        )
        
        # 探索状态
        self.maze_rect = None
        self.exit_position = None
        self.best_frontiers = []
        
        # 初始化UI
        self.init_ui()
        
        # 启动定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(config.UPDATE_INTERVAL)
        
        # 确保map_visualizer已初始化
        if not hasattr(self, 'map_visualizer'):
            self.log("警告：map_visualizer未正确初始化")
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('建图窗口 - 迷宫导航机器人')
        self.setGeometry(200, 200, 1000, 800)
        
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, stretch=1)
        
        # 右侧地图显示
        right_panel = self.create_map_panel()
        main_layout.addWidget(right_panel, stretch=4)
        
        # 连接信号（在所有面板创建完成后）
        self.connect_signals()
        
    def create_control_panel(self):
        """创建左侧控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 1. 地图控制组
        map_group = QGroupBox("地图控制")
        map_layout = QVBoxLayout()
        
        self.btn_load_map = QPushButton("加载地图")
        self.btn_clear_map = QPushButton("清空地图")
        self.btn_save_map = QPushButton("保存地图")
        
        map_layout.addWidget(self.btn_load_map)
        map_layout.addWidget(self.btn_clear_map)
        map_layout.addWidget(self.btn_save_map)
        
        map_group.setLayout(map_layout)
        layout.addWidget(map_group)
        
        # 2. 探索控制组
        explore_group = QGroupBox("探索控制")
        explore_layout = QVBoxLayout()
        
        self.btn_start_exploration = QPushButton("开始探索")
        self.btn_stop_exploration = QPushButton("停止探索")
        self.btn_stop_exploration.setEnabled(False)
        
        explore_layout.addWidget(self.btn_start_exploration)
        explore_layout.addWidget(self.btn_stop_exploration)
        
        explore_group.setLayout(explore_layout)
        layout.addWidget(explore_group)
        
        # 3. 导航控制组
        nav_group = QGroupBox("导航控制")
        nav_layout = QVBoxLayout()
        
        # 坐标输入
        coord_layout = QGridLayout()
        coord_layout.addWidget(QLabel("X(mm):"), 0, 0)
        self.input_target_x = QSpinBox()
        self.input_target_x.setRange(0, 5000)
        self.input_target_x.setValue(2500)
        self.input_target_x.setSingleStep(100)
        coord_layout.addWidget(self.input_target_x, 0, 1)
        
        coord_layout.addWidget(QLabel("Y(mm):"), 1, 0)
        self.input_target_y = QSpinBox()
        self.input_target_y.setRange(0, 5000)
        self.input_target_y.setValue(2500)
        self.input_target_y.setSingleStep(100)
        coord_layout.addWidget(self.input_target_y, 1, 1)
        nav_layout.addLayout(coord_layout)
        
        self.btn_go_to_coord = QPushButton("前往坐标")
        self.btn_return_home = QPushButton("返回起点")
        
        nav_layout.addWidget(self.btn_go_to_coord)
        nav_layout.addWidget(self.btn_return_home)
        
        # 出口设置
        nav_layout.addWidget(QLabel("─" * 20))
        nav_layout.addWidget(QLabel("出口设置:"))
        
        exit_layout = QHBoxLayout()
        self.btn_set_exit = QPushButton("设置出口")
        self.btn_go_to_exit = QPushButton("前往出口")
        exit_layout.addWidget(self.btn_set_exit)
        exit_layout.addWidget(self.btn_go_to_exit)
        nav_layout.addLayout(exit_layout)
        
        nav_group.setLayout(nav_layout)
        layout.addWidget(nav_group)
        
        # 4. 状态显示组
        status_group = QGroupBox("状态信息")
        status_layout = QVBoxLayout()
        
        self.lbl_pose = QLabel("位置: X=0.00 Y=0.00\n朝向: 0.0°")
        self.lbl_pose.setFont(QFont("Courier", 9))
        status_layout.addWidget(self.lbl_pose)
        
        self.lbl_progress = QLabel("探索进度: 0%")
        status_layout.addWidget(self.lbl_progress)
        
        self.lbl_frontier_status = QLabel("前沿: 0个")
        self.lbl_frontier_status.setFont(QFont("Courier", 8))
        status_layout.addWidget(self.lbl_frontier_status)
        
        self.lbl_exit_status = QLabel("出口: 未设置")
        self.lbl_exit_status.setFont(QFont("Courier", 8))
        status_layout.addWidget(self.lbl_exit_status)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        return panel
    
    def create_map_panel(self):
        """创建右侧地图面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 地图可视化
        self.map_visualizer = MapVisualizer(self.map_manager)
        layout.addWidget(self.map_visualizer)
        
        return panel
    
    def connect_signals(self):
        """连接信号和槽"""
        # 地图控制
        self.btn_load_map.clicked.connect(self.load_map)
        self.btn_clear_map.clicked.connect(self.clear_map)
        self.btn_save_map.clicked.connect(self.save_map)
        
        # 探索控制
        self.btn_start_exploration.clicked.connect(self.start_exploration)
        self.btn_stop_exploration.clicked.connect(self.stop_exploration)
        
        # 导航控制
        self.btn_go_to_coord.clicked.connect(self.go_to_coord)
        self.btn_return_home.clicked.connect(self.return_home)
        self.btn_set_exit.clicked.connect(self.on_set_exit)
        self.btn_go_to_exit.clicked.connect(self.go_to_exit)
        
        # 地图点击事件
        self.map_visualizer.point_clicked.connect(self.on_map_clicked)
    
    def update_robot_pose(self, x, y, theta):
        """更新机器人位姿"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        
        # 记录轨迹
        self.trajectory.append((x, y))
        if len(self.trajectory) > 1000:
            self.trajectory.pop(0)
        
        # 更新地图可视化中的机器人位置
        if hasattr(self, 'map_visualizer') and self.map_visualizer is not None:
            self.map_visualizer.set_robot_pose(x, y, theta)
            self.map_visualizer.set_trajectory(self.trajectory)
            # 强制更新显示
            self.map_visualizer.update(force_full_redraw=True)
        
        # 更新显示
        self.lbl_pose.setText(f"位置: X={x:.2f} Y={y:.2f}\n朝向: {theta:.1f}°")
    
    def update_lidar_data(self, lidar_data):
        """更新雷达数据"""
        # 始终更新地图，不依赖探索模式
        # 更新地图
        self.map_manager.update_from_lidar(
            self.robot_x, self.robot_y, self.robot_theta, lidar_data
        )
        # 更新地图可视化
        if hasattr(self, 'map_visualizer') and self.map_visualizer is not None:
            self.map_visualizer.update()
    
    def load_map(self):
        """加载地图文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择地图文件", "../submission", "JSON Files (*.json)"
        )
        
        if file_path:
            result = self.map_manager.load_from_json(file_path)
            if result[0]:
                start_pos, start_grid = result
                self.log(f"成功加载地图: {os.path.basename(file_path)}")
                self.log(f"起点: ({start_pos[0]:.1f}, {start_pos[1]:.1f})")
                self.map_visualizer.update()
            else:
                self.log("加载地图失败")
    
    def clear_map(self):
        """清空地图"""
        self.map_manager.grid_map.fill(config.MAP_STATE_UNKNOWN)
        self.trajectory.clear()
        self.current_path = None
        self.map_visualizer.update()
        self.log("地图已清空")
    
    def save_map(self):
        """保存地图"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存地图文件", "../submission", "JSON Files (*.json)"
        )
        
        if file_path:
            if self.map_manager.save_to_json(file_path):
                self.log(f"地图已保存: {os.path.basename(file_path)}")
            else:
                self.log("保存地图失败")
    
    def start_exploration(self):
        """开始探索"""
        self.exploration_mode = True
        self.exploration_started.emit()
        
        # 初始化机器人周围的区域为空闲
        self.map_manager.initialize_robot_area(self.robot_x, self.robot_y, radius=600)
        
        # 更新地图可视化
        self.map_visualizer.update(force_full_redraw=True)
        
        self.btn_start_exploration.setEnabled(False)
        self.btn_stop_exploration.setEnabled(True)
        self.log("开始自主探索模式")
    
    def stop_exploration(self):
        """停止探索"""
        self.exploration_mode = False
        self.exploration_stopped.emit()
        
        self.btn_start_exploration.setEnabled(True)
        self.btn_stop_exploration.setEnabled(False)
        self.log("停止探索模式")
    
    def go_to_coord(self):
        """前往输入的坐标"""
        # 停止探索模式
        if self.exploration_mode:
            self.stop_exploration()
        
        # 获取输入的坐标
        target_x = self.input_target_x.value()
        target_y = self.input_target_y.value()
        
        self.log(f"前往坐标: ({target_x}, {target_y})")
        
        # 规划路径
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        target_grid = self.map_manager.world_to_grid(target_x, target_y)
        
        path = self.path_planner.a_star_safe(robot_grid, target_grid)
        if path:
            smoothed_path = self.path_planner.smooth_path(path)
            self.current_path = smoothed_path
            self.goal_position = (target_x, target_y)
            self.log(f"规划路径成功: {len(path)}点 → 简化后{len(smoothed_path)}点")
        else:
            self.log("无法规划路径到该坐标")
    
    def return_home(self):
        """返回起点"""
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        home_grid = self.map_manager.world_to_grid(config.ROBOT_INIT_X, config.ROBOT_INIT_Y)
        
        path = self.path_planner.a_star_safe(robot_grid, home_grid)
        if path:
            smoothed_path = self.path_planner.smooth_path(path)
            self.current_path = smoothed_path
            self.log(f"规划返回路径: {len(path)}点 → 简化后{len(smoothed_path)}点")
        else:
            self.log("无法规划返回路径")
    
    def on_set_exit(self):
        """设置出口位置"""
        from PyQt5.QtWidgets import QInputDialog
        
        world_x, ok1 = QInputDialog.getInt(self, "设置出口", "出口X坐标(mm):",
                                           value=3500, min=0, max=5000, step=100)
        if not ok1:
            return
        
        world_y, ok2 = QInputDialog.getInt(self, "设置出口", "出口Y坐标(mm):",
                                           value=3500, min=0, max=5000, step=100)
        if not ok2:
            return
        
        self.exit_position = self.map_manager.world_to_grid(world_x, world_y)
        
        # 更新可视化
        self.map_visualizer.set_exit_position(self.exit_position)
        self.map_visualizer.update()
        
        # 更新状态标签
        self.lbl_exit_status.setText(f"出口: ({world_x}, {world_y})")
        self.lbl_exit_status.setStyleSheet("color: lime; font-weight: bold;")
        
        self.log(f"设置出口位置: ({world_x}, {world_y})")
    
    def go_to_exit(self):
        """前往出口"""
        if self.exit_position is None:
            self.log("未设置出口位置")
            return
        
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        path = self.path_planner.a_star_safe(robot_grid, self.exit_position)
        
        if path:
            smoothed_path = self.path_planner.smooth_path(path)
            self.current_path = smoothed_path
            self.log(f"规划到出口路径: {len(path)}点 → 简化后{len(smoothed_path)}点")
        else:
            self.log("无法规划到出口的路径")
    
    def on_map_clicked(self, world_x, world_y):
        """地图点击事件"""
        # 停止探索模式
        if self.exploration_mode:
            self.stop_exploration()
        
        self.log(f"点击位置: ({world_x:.1f}, {world_y:.1f})")
        
        # 规划路径到点击位置
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        goal_grid = self.map_manager.world_to_grid(world_x, world_y)
        
        path = self.path_planner.a_star_safe(robot_grid, goal_grid)
        if path:
            self.current_path = path
            self.log(f"规划路径成功，共 {len(path)} 个点")
            self.goal_position = (world_x, world_y)
        else:
            self.log("无法规划路径到该位置")
        
        # 发送信号给主窗口
        self.map_clicked.emit(world_x, world_y)
    
    def update_display(self):
        """定时更新显示"""
        import time
        
        current_time = time.time() * 1000
        
        # 更新探索进度
        progress = self.map_manager.get_exploration_progress()
        self.lbl_progress.setText(f"探索进度: {progress:.1f}%")
        
        # 更新地图显示
        if not hasattr(self, 'last_map_update'):
            self.last_map_update = 0
        
        if current_time - self.last_map_update >= config.MAP_UPDATE_INTERVAL:
            self.map_visualizer.set_robot_pose(self.robot_x, self.robot_y, self.robot_theta)
            self.map_visualizer.set_trajectory(self.trajectory)
            self.map_visualizer.set_path(self.current_path)
            self.map_visualizer.update()
            self.last_map_update = current_time
        
        # 探索模式逻辑
        if self.exploration_mode:
            if current_time - self.last_exploration_time >= config.EXPLORATION_INTERVAL:
                self.exploration_step()
                self.last_exploration_time = current_time
    
    def exploration_step(self):
        """探索步骤"""
        import time
        
        current_time = time.time() * 1000
        
        # 等待雷达数据累积
        if not hasattr(self, 'exploration_start_time'):
            self.exploration_start_time = current_time
        
        wait_time = 10000  # 10秒
        if current_time - self.exploration_start_time < wait_time:
            return
        
        # 如果当前有路径正在执行，先执行完
        if self.current_path and len(self.current_path) > 0:
            return
        
        # 更新迷宫ROI
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        self.maze_rect = self.map_rectifier.update_from_grid(
            self.map_manager.grid_map,
            ensure_contains=robot_grid
        )
        
        # 检测前沿点
        frontiers = self.frontier_explorer.detect_frontiers(
            self.map_manager.grid_map,
            maze_rect=self.maze_rect
        )
        
        progress = self.map_manager.get_exploration_progress()
        
        if not frontiers:
            if progress < 5.0:
                return
            
            self.log("探索完成！未发现更多前沿")
            self.log(f"地图探索进度: {progress:.1f}%")
            self.stop_exploration()
            return
        
        # 更新前沿点可视化
        self.map_visualizer.set_frontiers(frontiers, [])
        self.map_visualizer.set_maze_rect(self.maze_rect)
        
        # 更新状态标签
        self.lbl_frontier_status.setText(f"前沿: {len(frontiers)}个")
        
        # 选择最佳前沿点
        self.best_frontiers = self.frontier_explorer.pick_best_frontiers(
            robot_grid,
            frontiers,
            self.map_manager.grid_map,
            goal_grid_pos=self.exit_position,
            maze_rect=self.maze_rect
        )
        
        # 更新最佳前沿点可视化
        self.map_visualizer.set_frontiers(frontiers, self.best_frontiers)
        
        if not self.best_frontiers:
            self.log("所有前沿点都不可达")
            return
        
        # 尝试规划到最佳前沿点
        nearest_frontier = self.best_frontiers[0]
        path = self.path_planner.a_star_safe(robot_grid, nearest_frontier, expansion_radius=3)
        
        if path and len(path) > 1:
            smoothed_path = self.path_planner.smooth_path(path, target_distance_cells=7)
            self.current_path = smoothed_path
            self.log(f"规划探索路径: {len(path)}点 → 简化后{len(smoothed_path)}点")
    
    def log(self, message):
        """添加日志"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] [建图窗口] {message}")


if __name__ == '__main__':
    from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    # 设置中文字体
    font = QFont()
    font.setFamily("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)
    
    window = MappingWindow()
    window.show()
    sys.exit(app.exec_())
