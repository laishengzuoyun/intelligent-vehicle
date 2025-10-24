"""
Maze Navigation Robot Main GUI
Main control interface for robot navigation system
"""

import sys
import os
import math
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QComboBox,
                             QTextEdit, QGroupBox, QGridLayout, QFileDialog,
                             QRadioButton, QButtonGroup, QSpinBox, QDoubleSpinBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import numpy as np

import config
from bluetooth_comm import BluetoothComm
from map_manager import OccupancyGridMap
from path_planner import PathPlanner
# 移除地图和雷达可视化导入，使用独立窗口
from simulator import VirtualRobotSimulator
from odometry import Odometry, SimpleOdometry
# 新增：集成planning.py的优化模块
from frontier_explorer import OptimizedFrontierExplorer
from goal_checker import GoalReachabilityChecker
from map_rectifier import MapRectifier
# 新增：智能避障控制器
from smart_controller import SmartObstacleController, SimpleManualController, ControllerConfig, Pose, LidarScan


def setup_chinese_font(app):
    """Configure Chinese font for Qt application

    Args:
        app: QApplication instance
    """
    font = QFont()
    font.setFamily("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()

        # 移动状态控制
        self.is_moving = False
        self.movement_start_time = 0
        
        # 仿真模式标志
        self.simulation_mode = False

        # 初始化组件
        self.bluetooth = BluetoothComm()
        self.simulator = VirtualRobotSimulator()
        self.map_manager = OccupancyGridMap(
            config.WORLD_WIDTH,
            config.WORLD_HEIGHT,
            config.CELL_SIZE
        )
        self.path_planner = PathPlanner(self.map_manager)

        # 里程计
        if config.ODOMETRY_MODE == "encoder":
            self.odometry = Odometry()
        else:
            self.odometry = SimpleOdometry()

        # 机器人状态（初始位置在地图中心，避免坐标变负）
        self.robot_x = config.ROBOT_INIT_X
        self.robot_y = config.ROBOT_INIT_Y
        self.robot_theta = config.ROBOT_INIT_THETA
        self.trajectory = []  # 轨迹记录
        
        # 坐标转换偏移量（用于STM32相对坐标到地图绝对坐标的转换）
        self._coord_offset_x = config.ROBOT_INIT_X
        self._coord_offset_y = config.ROBOT_INIT_Y
        
        # 路径规划
        self.current_path = None
        self.goal_position = None
        
        # 删除探索模式相关变量
        # self.exploration_mode = False
        self.last_exploration_time = 0  # 上次探索时间戳
        self.exploration_start_time = 0  # 探索开始时间
        self.last_map_update_time = 0  # 上次地图可视化更新时间

        # 优化的探索模块
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
        self.maze_rect = None  # 迷宫有效区域
        self.exit_position = None  # 出口位置（栅格坐标）
        self.best_frontiers = []  # 排序后的前沿点列表

        # 旋转状态跟踪（防止旋转时地图更新导致坐标系偏移）
        self.is_rotating = False
        self.last_rotation_command_time = 0

        # 性能优化：日志缓冲
        self._log_buffer = []  # 日志缓冲区
        self._last_log_update_time = 0  # 上次日志更新时间
        
        # BreezeSLAM建图器将在UI初始化后创建
        self.breezeslam_mapper = None
        self.use_breezeslam = False
        
        # 性能监控
        self._performance_stats = {
            'update_count': 0,
            'last_fps_time': 0,
            'fps': 0,
            'map_updates': 0,
            'lidar_updates': 0,
            'log_updates': 0,
            'ray_calculations': 0,
            'lidar_points_processed': 0
        }
        
        # 避障检测相关
        self._obstacle_check_timer = QTimer()
        self._obstacle_check_timer.timeout.connect(self.check_obstacle_continuously)
        self._obstacle_check_timer.setInterval(5)  # 每5ms检查一次，提高响应速度
        self._manual_control_active = False
        
        # 智能避障控制器
        controller_config = ControllerConfig()
        controller_config.lookahead_mm = 450.0  # 前瞻距离450mm
        controller_config.k_ang = 2.0  # 角度控制增益
        controller_config.k_avoid = 1.5  # 避障增益
        controller_config.avoid_radius_mm = 350.0  # 避障半径350mm
        controller_config.v_max_mmps = 250.0  # 最大速度250mm/s
        controller_config.repulse_gain = 1.0  # 排斥力增益
        controller_config.min_obstacle_distance_mm = 300.0  # 最小障碍物距离300mm
        self.smart_controller = SmartObstacleController(controller_config)
        self.manual_controller = SimpleManualController(controller_config)
        self.use_smart_controller = True  # 默认使用智能控制器
        
        # 初始化UI
        self.init_ui()
        
        # 连接信号
        self.connect_signals()
        
        # 启动定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(config.UPDATE_INTERVAL)

        # 延迟刷新端口列表，确保所有组件都已初始化
        QTimer.singleShot(100, self.refresh_ports)
        
        # 注释掉自动初始化机器人区域，只在真正需要时初始化
        # self.map_manager.initialize_robot_area(self.robot_x, self.robot_y, radius=300)
        # self.log("初始化机器人周围300mm区域为空闲")
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle('迷宫导航机器人控制系统 v1.0 - 简化版')
        self.setGeometry(100, 100, 600, 800)
        
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        # 添加独立窗口按钮
        window_panel = self.create_window_panel()
        main_layout.addWidget(window_panel)
        
    def create_control_panel(self):
        """创建控制面板 - 优化多列布局"""
        panel = QWidget()
        main_layout = QHBoxLayout(panel)  # 改为水平布局，支持多列
        
        # 左侧列：连接和模式控制
        left_column = QVBoxLayout()

        # 1. 连接控制组
        conn_group = QGroupBox("蓝牙连接")
        conn_layout = QVBoxLayout()

        # 模式选择
        mode_layout = QHBoxLayout()
        self.btn_real_mode = QPushButton("[实物模式]")
        self.btn_sim_mode = QPushButton("[仿真模式]")
        self.btn_real_mode.setCheckable(True)
        self.btn_sim_mode.setCheckable(True)
        self.btn_real_mode.setChecked(True)
        mode_layout.addWidget(self.btn_real_mode)
        mode_layout.addWidget(self.btn_sim_mode)
        conn_layout.addLayout(mode_layout)

        self.port_combo = QComboBox()
        # 先不刷新端口，等log_text创建后再刷新
        conn_layout.addWidget(QLabel("串口:"))
        conn_layout.addWidget(self.port_combo)
        
        btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("连接")
        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setEnabled(False)
        btn_layout.addWidget(self.btn_connect)
        btn_layout.addWidget(self.btn_disconnect)
        conn_layout.addLayout(btn_layout)
        
        self.btn_refresh = QPushButton("刷新端口")
        conn_layout.addWidget(self.btn_refresh)
        
        self.lbl_status = QLabel("未连接")
        self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self.lbl_status)
        
        conn_group.setLayout(conn_layout)
        left_column.addWidget(conn_group)
        
        # 2. 手动控制组
        control_group = QGroupBox("手动控制")
        control_layout = QGridLayout()
        
        self.btn_forward = QPushButton("↑ 前进")
        self.btn_backward = QPushButton("↓ 后退")
        self.btn_left = QPushButton("← 左转")
        self.btn_right = QPushButton("→ 右转")
        self.btn_stop = QPushButton("■ 停止")
        self.btn_stop.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold;")
        
        control_layout.addWidget(self.btn_forward, 0, 1)
        control_layout.addWidget(self.btn_left, 1, 0)
        control_layout.addWidget(self.btn_stop, 1, 1)
        control_layout.addWidget(self.btn_right, 1, 2)
        control_layout.addWidget(self.btn_backward, 2, 1)
        
        control_group.setLayout(control_layout)
        
        # 中间列：手动控制
        middle_column = QVBoxLayout()
        middle_column.addWidget(control_group)
        middle_column.addStretch()
        main_layout.addLayout(middle_column)
        
        # 3. 雷达控制组
        radar_group = QGroupBox("雷达控制")
        radar_layout = QVBoxLayout()
        
        self.btn_start_radar = QPushButton("启动雷达")
        self.btn_stop_radar = QPushButton("停止雷达")
        self.btn_init_area = QPushButton("初始化机器人区域")
        
        radar_layout.addWidget(self.btn_start_radar)
        radar_layout.addWidget(self.btn_stop_radar)
        radar_layout.addWidget(self.btn_init_area)
        
        radar_group.setLayout(radar_layout)
        left_column.addWidget(radar_group)
        
        # 3. 初始位置设置组
        init_pos_group = QGroupBox("初始位置设置")
        init_pos_layout = QVBoxLayout()
        
        # X坐标设置
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X坐标:"))
        self.input_init_x = QSpinBox()
        self.input_init_x.setRange(0, 5000)
        self.input_init_x.setValue(config.ROBOT_INIT_X)
        self.input_init_x.setSuffix(" mm")
        x_layout.addWidget(self.input_init_x)
        init_pos_layout.addLayout(x_layout)
        
        # Y坐标设置
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y坐标:"))
        self.input_init_y = QSpinBox()
        self.input_init_y.setRange(0, 5000)
        self.input_init_y.setValue(config.ROBOT_INIT_Y)
        self.input_init_y.setSuffix(" mm")
        y_layout.addWidget(self.input_init_y)
        init_pos_layout.addLayout(y_layout)
        
        # 朝向设置
        theta_layout = QHBoxLayout()
        theta_layout.addWidget(QLabel("朝向:"))
        self.input_init_theta = QSpinBox()
        self.input_init_theta.setRange(0, 359)
        self.input_init_theta.setValue(config.ROBOT_INIT_THETA)
        self.input_init_theta.setSuffix(" °")
        theta_layout.addWidget(self.input_init_theta)
        init_pos_layout.addLayout(theta_layout)
        
        # 设置按钮
        self.btn_set_init_pos = QPushButton("设置初始位置")
        init_pos_layout.addWidget(self.btn_set_init_pos)
        
        # 重置按钮
        self.btn_reset_init_pos = QPushButton("重置为默认")
        init_pos_layout.addWidget(self.btn_reset_init_pos)
        
        init_pos_group.setLayout(init_pos_layout)
        left_column.addWidget(init_pos_group)
        
        # 4. 窗口控制组
        window_group = QGroupBox("窗口控制")
        window_layout = QVBoxLayout()
        
        self.btn_open_mapping = QPushButton("🗺️ 打开建图窗口")
        self.btn_open_radar = QPushButton("📡 打开雷达查看器")
        
        window_layout.addWidget(self.btn_open_mapping)
        window_layout.addWidget(self.btn_open_radar)
        
        window_group.setLayout(window_layout)
        left_column.addWidget(window_group)
        
        left_column.addStretch()
        main_layout.addLayout(left_column)
        
        # 4. 导航控制组
        nav_group = QGroupBox("导航控制")
        nav_layout = QVBoxLayout()

        # 删除探索控制按钮
        # self.btn_start_exploration = QPushButton("开始探索")
        # self.btn_stop_exploration = QPushButton("停止探索")
        # nav_layout.addWidget(self.btn_start_exploration)
        # nav_layout.addWidget(self.btn_stop_exploration)

        # 分隔线
        nav_layout.addWidget(QLabel("─" * 20))

        # 手动导航
        nav_layout.addWidget(QLabel("手动导航:"))

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

        self.btn_go_to_coord = QPushButton("▶ 前往坐标")
        nav_layout.addWidget(self.btn_go_to_coord)

        # 导航模式切换按钮
        nav_mode_layout = QHBoxLayout()
        nav_mode_layout.addWidget(QLabel("导航模式:"))
        self.btn_nav_mode = QPushButton("走走停停")
        self.btn_nav_mode.setCheckable(True)
        self.btn_nav_mode.setChecked(True)  # 默认启用走走停停模式
        self.btn_nav_mode.clicked.connect(self.toggle_navigation_mode)
        nav_mode_layout.addWidget(self.btn_nav_mode)
        nav_layout.addLayout(nav_mode_layout)

        # BreezeSLAM切换按钮
        slam_layout = QHBoxLayout()
        slam_layout.addWidget(QLabel("建图算法:"))
        self.btn_slam_mode = QPushButton("BreezeSLAM")
        self.btn_slam_mode.setCheckable(True)
        self.btn_slam_mode.setChecked(self.use_breezeslam)  # 根据初始化状态设置
        self.btn_slam_mode.clicked.connect(self.toggle_slam_mode)
        slam_layout.addWidget(self.btn_slam_mode)
        nav_layout.addLayout(slam_layout)

        self.btn_return_home = QPushButton("🏠 返回起点")
        nav_layout.addWidget(self.btn_return_home)

        # 删除验收考试相关按钮
        # nav_layout.addWidget(QLabel("─" * 20))
        # nav_layout.addWidget(QLabel("验收考试:"))
        # exit_layout = QHBoxLayout()
        # self.btn_set_exit = QPushButton("设置出口")
        # self.btn_go_to_exit = QPushButton("前往出口")
        # exit_layout.addWidget(self.btn_set_exit)
        # exit_layout.addWidget(self.btn_go_to_exit)
        # nav_layout.addLayout(exit_layout)

        nav_group.setLayout(nav_layout)
        
        # 右侧列：导航和高级功能
        right_column = QVBoxLayout()
        right_column.addWidget(nav_group)
        
        # 5. 机器人状态显示
        status_group = QGroupBox("机器人状态")
        status_layout = QVBoxLayout()
        
        self.lbl_pose = QLabel("位置: X=0.00 Y=0.00\n朝向: 0.0°")
        self.lbl_pose.setFont(QFont("Courier", 9))
        status_layout.addWidget(self.lbl_pose)
        
        # 删除探索相关状态显示
        # self.lbl_progress = QLabel("探索进度: 0%")
        # status_layout.addWidget(self.lbl_progress)
        #
        # self.lbl_frontier_status = QLabel("前沿: 0个")
        # self.lbl_frontier_status.setFont(QFont("Courier", 8))
        # status_layout.addWidget(self.lbl_frontier_status)
        #
        # self.lbl_exit_status = QLabel("出口: 未设置")
        # self.lbl_exit_status.setFont(QFont("Courier", 8))
        # status_layout.addWidget(self.lbl_exit_status)
        
        status_group.setLayout(status_layout)
        right_column.addWidget(status_group)
        
        # 6. 日志输出
        log_group = QGroupBox("日志输出")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)
        
        self.btn_clear_log = QPushButton("清空日志")
        log_layout.addWidget(self.btn_clear_log)
        
        self.btn_performance = QPushButton("性能监控")
        log_layout.addWidget(self.btn_performance)
        
        # 智能控制器切换按钮
        self.btn_controller = QPushButton("切换控制器")
        self.btn_controller.clicked.connect(self.toggle_controller)
        log_layout.addWidget(self.btn_controller)
        
        log_group.setLayout(log_layout)
        right_column.addWidget(log_group)
        
        right_column.addStretch()
        main_layout.addLayout(right_column)
        
        return panel
        
        # 性能优化：日志缓冲
        self._log_buffer = []  # 日志缓冲区
        self._last_log_update_time = 0  # 上次日志更新时间
        
        # 性能监控
        self._performance_stats = {
            'update_count': 0,
            'last_fps_time': 0,
            'fps': 0,
            'map_updates': 0,
            'lidar_updates': 0,
            'log_updates': 0,
            'ray_calculations': 0,
            'lidar_points_processed': 0
        }
        
        # 避障检测相关
        self._obstacle_check_timer = QTimer()
        self._obstacle_check_timer.timeout.connect(self.check_obstacle_continuously)
        self._obstacle_check_timer.setInterval(5)  # 每5ms检查一次，提高响应速度
        self._manual_control_active = False
        
        # 注释掉不合适的初始化，只在真正需要时初始化
        # self.map_manager.initialize_robot_area(self.robot_x, self.robot_y, radius=300)
        # self.log("初始化机器人周围300mm区域为空闲")
        
        layout.addStretch()
        
        return panel
    
    def create_window_panel(self):
        """创建独立窗口面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        window_group = QGroupBox("独立窗口")
        window_layout = QVBoxLayout()
        
        # 注意：这些按钮已经在create_control_panel中创建了
        # 这里只需要连接信号即可
        window_layout.addWidget(self.btn_open_mapping)
        window_layout.addWidget(self.btn_open_radar)
        
        window_group.setLayout(window_layout)
        layout.addWidget(window_group)
        
        return panel
    
    def connect_signals(self):
        """连接信号和槽"""
        # 模式切换 - 使用toggled信号而不是clicked，避免重复触发
        self.btn_real_mode.toggled.connect(lambda checked: self.switch_to_real_mode() if checked else None)
        self.btn_sim_mode.toggled.connect(lambda checked: self.switch_to_sim_mode() if checked else None)

        # 蓝牙连接
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        
        # 手动控制
        self.btn_forward.clicked.connect(self.on_forward_button)
        self.btn_stop.clicked.connect(self.on_stop_button)
        self.btn_left.clicked.connect(self.on_left_button)
        self.btn_right.clicked.connect(self.on_right_button)
        self.btn_backward.clicked.connect(self.on_backward_button)
        
        # 雷达控制
        self.btn_start_radar.clicked.connect(self.start_radar)
        self.btn_stop_radar.clicked.connect(self.stop_radar)
        self.btn_init_area.clicked.connect(self.init_robot_area)
        
        # 初始位置设置
        self.btn_set_init_pos.clicked.connect(self.set_initial_position)
        self.btn_reset_init_pos.clicked.connect(self.reset_initial_position)
        
        # 删除探索按钮信号连接
        # self.btn_start_exploration.clicked.connect(self.start_exploration)
        # self.btn_stop_exploration.clicked.connect(self.stop_exploration)
        self.btn_return_home.clicked.connect(self.return_home)
        self.btn_go_to_coord.clicked.connect(self.go_to_coord)
        # 删除验收考试按钮信号连接
        # self.btn_set_exit.clicked.connect(self.on_set_exit)
        # self.btn_go_to_exit.clicked.connect(self.go_to_exit)
        
        # 日志
        self.btn_clear_log.clicked.connect(self.log_text.clear)
        self.btn_performance.clicked.connect(self.show_performance_stats)
        
        # 蓝牙数据接收 - 添加错误处理
        try:
            self.bluetooth.data_received.connect(self.on_data_received)
            self.bluetooth.connection_changed.connect(self.on_connection_changed)
            self.bluetooth.pose_updated.connect(self.on_pose_updated)
            self.bluetooth.encoder_data_updated.connect(self.on_encoder_data_updated)
            self.bluetooth.lidar_data_updated.connect(self.on_lidar_data_updated)
        except Exception as e:
            print(f"蓝牙信号连接失败: {e}")

        # 模拟器数据接收 - 添加错误处理
        try:
            self.simulator.pose_updated.connect(self.on_pose_updated)
            self.simulator.encoder_data_updated.connect(self.on_encoder_data_updated)
            self.simulator.lidar_data_updated.connect(self.on_lidar_data_updated)
            self.simulator.data_received.connect(self.on_data_received)
        except Exception as e:
            print(f"模拟器信号连接失败: {e}")

        # 独立窗口
        self.btn_open_mapping.clicked.connect(self.open_mapping_window)
        self.btn_open_radar.clicked.connect(self.open_radar_viewer)
        
        # 初始化BreezeSLAM建图器（UI创建完成后）
        self._init_breezeslam()

    def _init_breezeslam(self):
        """初始化BreezeSLAM建图器"""
        try:
            from breezeslam_mapper import BreezeSLAMMapper
            self.breezeslam_mapper = BreezeSLAMMapper(
                map_size_pixels=500,
                map_size_meters=50
            )
            self.use_breezeslam = True
            self.log("BreezeSLAM建图器已启用")
        except ImportError as e:
            self.breezeslam_mapper = None
            self.use_breezeslam = False
            self.log(f"BreezeSLAM不可用，使用传统建图: {e}")

    def switch_to_real_mode(self):
        """切换到实物模式"""
        # 防止重复触发
        if not self.simulation_mode:
            return

        self.simulation_mode = False

        # 阻塞信号，避免循环触发
        self.btn_real_mode.blockSignals(True)
        self.btn_sim_mode.blockSignals(True)
        self.btn_real_mode.setChecked(True)
        self.btn_sim_mode.setChecked(False)
        self.btn_real_mode.blockSignals(False)
        self.btn_sim_mode.blockSignals(False)

        # 停止模拟器
        if self.simulator.isRunning():
            self.simulator.stop()

        # 启用蓝牙控件
        self.port_combo.setEnabled(True)
        self.btn_connect.setEnabled(True)
        self.btn_refresh.setEnabled(True)

        self.log("[OK] 切换到实物模式")

    def switch_to_sim_mode(self):
        """切换到仿真模式"""
        # 防止重复触发
        if self.simulation_mode:
            return

        self.simulation_mode = True

        # 阻塞信号，避免循环触发
        self.btn_real_mode.blockSignals(True)
        self.btn_sim_mode.blockSignals(True)
        self.btn_real_mode.setChecked(False)
        self.btn_sim_mode.setChecked(True)
        self.btn_real_mode.blockSignals(False)
        self.btn_sim_mode.blockSignals(False)

        # 断开蓝牙
        if self.bluetooth.is_connected:
            self.bluetooth.disconnect()

        # 禁用蓝牙控件
        self.port_combo.setEnabled(False)
        self.btn_connect.setEnabled(False)
        self.btn_refresh.setEnabled(False)

        # 启动模拟器
        if not self.simulator.isRunning():
            self.simulator.start()
            import time
            time.sleep(0.1)  # 等待线程启动
            if self.simulator.isRunning():
                self.log("仿真器已启动")
            else:
                self.log("[ERROR] 仿真器启动失败！")
        else:
            self.log("仿真器已经在运行")

        self.log("[OK] 切换到仿真模式")
        self.log("[SIM] 虚拟小车已启动，无需硬件即可测试所有功能")
        self.lbl_status.setText("仿真模式")
        self.lbl_status.setStyleSheet("color: blue; font-weight: bold;")

    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = BluetoothComm.list_ports()
        self.port_combo.addItems(ports)
        self.log(f"找到 {len(ports)} 个串口")

    def on_connect(self):
        """连接蓝牙"""
        port = self.port_combo.currentText()
        if not port:
            self.log("请选择串口")
            return

        self.log(f"正在连接到 {port}...")
        if self.bluetooth.connect(port):
            self.log(f"成功连接到 {port}")
        else:
            self.log(f"连接失败")

    def on_disconnect(self):
        """断开蓝牙"""
        self.bluetooth.disconnect()
        self.log("已断开连接")

    def on_connection_changed(self, connected):
        """连接状态改变"""
        if connected:
            self.lbl_status.setText("已连接")
            self.lbl_status.setStyleSheet("color: green; font-weight: bold;")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
        else:
            self.lbl_status.setText("未连接")
            self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)

    def on_forward_button(self):
        """前进按钮 - 智能前进控制版本"""
        # 检查是否已经在智能前进模式
        if hasattr(self, '_smart_forward_active') and self._smart_forward_active:
            self.log("⚠️ 智能前进模式已激活，请等待完成")
            return
        
        # 启动智能前进模式
        self._smart_forward_active = True
        self.log("🚀 启动智能前进模式：0.2秒前进 + 避障检测")
        
        # 开始第一步前进
        self._start_forward_step()

    def _start_forward_step(self):
        """开始前进步骤：0.2秒前进"""
        # 设置移动状态，停止建图
        self.is_moving = True
        self.movement_start_time = time.time()
        
        # 发送前进命令
        if self.bluetooth and self.bluetooth.is_connected:
            self.bluetooth.send_car_command(0)  # 0=前进
            self.log("🚀 前进0.2秒...")
        elif self.simulator and self.simulator.isRunning():
            self.simulator.send_command('0')  # 0=前进
            self.log("🚀 仿真器前进0.2秒...")
        else:
            self.log("⚠️ 请先连接设备或启动仿真器")
            self._smart_forward_active = False
            return
        
        # 设置0.2秒后停止的定时器
        if not hasattr(self, '_forward_step_timer'):
            self._forward_step_timer = QTimer()
            self._forward_step_timer.timeout.connect(self._stop_and_check_obstacle)
        
        self._forward_step_timer.start(200)  # 0.2秒 = 200ms

    def _stop_and_check_obstacle(self):
        """停止前进并检查障碍物"""
        # 停止前进
        if self.bluetooth and self.bluetooth.is_connected:
            self.bluetooth.send_car_command(1)  # 1=停止
        elif self.simulator and self.simulator.isRunning():
            self.simulator.send_command('1')  # 1=停止
        
        # 清除移动状态，恢复建图
        self.is_moving = False
        
        # 停止定时器
        if hasattr(self, '_forward_step_timer'):
            self._forward_step_timer.stop()
        
        self.log("⏹️ 停止前进，开始避障检测...")
        
        # 延迟一点时间让雷达数据稳定，然后检查障碍物
        QTimer.singleShot(500, self._check_obstacle_and_continue)

    def _check_obstacle_and_continue(self):
        """检查障碍物并决定是否继续前进"""
        if not self._smart_forward_active:
            return
        
        # 检查前方是否有障碍物
        obstacle_detected = self._check_front_obstacle()
        
        if obstacle_detected:
            # 有障碍物，停止智能前进
            self._smart_forward_active = False
            self.log("⚠️ 检测到前方障碍物，停止智能前进")
        else:
            # 无障碍物，继续前进
            self.log("✅ 前方安全，继续前进...")
            QTimer.singleShot(1000, self._start_forward_step)  # 1秒后继续

    def _check_front_obstacle(self):
        """检查前方是否有障碍物"""
        if not hasattr(self, 'map_manager') or self.map_manager is None:
            return False
        
        # 获取机器人当前位置
        robot_x, robot_y = self.robot_x, self.robot_y
        robot_theta = self.robot_theta
        
        # 检查前方一定距离内是否有障碍物
        check_distance = 300  # 检查前方300mm
        check_width = 200     # 检查宽度200mm
        
        # 计算检查区域
        rad = math.radians(robot_theta)
        
        # 前方中心点
        front_x = robot_x + check_distance * math.cos(rad)
        front_y = robot_y + check_distance * math.sin(rad)
        
        # 检查前方区域
        for offset in range(-check_width//2, check_width//2, 50):
            # 计算偏移点
            offset_rad = math.radians(robot_theta + 90)  # 垂直方向
            check_x = front_x + offset * math.cos(offset_rad)
            check_y = front_y + offset * math.sin(offset_rad)
            
            # 转换为栅格坐标
            grid_x, grid_y = self.map_manager.world_to_grid(check_x, check_y)
            
            # 检查是否为障碍物
            if (0 <= grid_x < self.map_manager.grid_map.shape[1] and 
                0 <= grid_y < self.map_manager.grid_map.shape[0]):
                if self.map_manager.grid_map[grid_y, grid_x] == config.MAP_STATE_OCCUPIED:
                    return True
        
        return False

    def _check_forward_progress(self):
        """检查前进进度，到达0.6秒后停下检测障碍物"""
        if not hasattr(self, '_forward_step_active') or not self._forward_step_active:
            return
            
        # 计算前进时间
        current_time = time.time()
        elapsed_time = current_time - self._forward_start_time
        
        # 根据小车速度计算应该前进的距离
        # 小车速度：50mm/s (0.05m/s)
        expected_distance = elapsed_time * 50  # mm
        
        # 同时检查实际位置变化（如果有位姿数据）
        actual_distance = 0
        if hasattr(self, '_forward_start_x') and hasattr(self, '_forward_start_y'):
            actual_distance = math.sqrt(
                (self.robot_x - self._forward_start_x) ** 2 + 
                (self.robot_y - self._forward_start_y) ** 2
            )
        
        # 使用时间控制为主，位置控制为辅
        # 0.6秒前进时间
        if elapsed_time >= 0.6:  # 0.6秒
            # 停止前进
            self.send_car_command(1)
            self._forward_step_timer.stop()
            self._forward_step_active = False
            
            self.log(f"✅ 前进时间{elapsed_time:.1f}秒（约{expected_distance:.1f}mm），停下检测障碍物...")
            if actual_distance > 0:
                self.log(f"📏 实际前进距离：{actual_distance:.1f}mm")
            
            # 检测前方障碍物
            self._detect_obstacles_after_step()
        else:
            # 继续前进
            self.log(f"📏 前进时间{elapsed_time:.1f}秒（约{expected_distance:.1f}mm）/0.6秒...")
            if actual_distance > 0:
                self.log(f"   实际前进距离：{actual_distance:.1f}mm")

    def _detect_obstacles_after_step(self):
        """前进步骤完成后检测障碍物 - 停止5秒等待避障检测"""
        self.log("🔍 停止5秒等待避障检测...")
        
        # 启动5秒等待定时器
        if not hasattr(self, '_scan_timer'):
            self._scan_timer = QTimer()
            self._scan_timer.timeout.connect(self._finish_obstacle_scan)
        
        self._scan_timer.start(5000)  # 5秒等待时间
        
    def _finish_obstacle_scan(self):
        """完成5秒等待，进行避障检测"""
        self._scan_timer.stop()
        
        self.log("🔍 等待完成，开始避障检测...")
        
        # 检查雷达数据是否可用
        lidar_data_available = False
        lidar_data = []
        
        if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
            if hasattr(self.radar_viewer, 'lidar_data'):
                lidar_data = self.radar_viewer.lidar_data
                if lidar_data and len(lidar_data) > 0:
                    lidar_data_available = True
        
        if lidar_data_available:
            # 使用雷达数据检测障碍物
            obstacle_detected = self.check_obstacle_from_lidar(lidar_data)
        else:
            # 使用地图检测障碍物
            obstacle_detected = self.check_obstacle_ahead()
        
        if obstacle_detected:
            self.log("⚠️ 检测到前方障碍物，停止前进")
            # 检测到障碍物，停止自动前进
            self._auto_forward_active = False
        else:
            self.log("✅ 前方无障碍物，继续前进0.6秒...")
            # 没有障碍物，自动继续前进
            self._auto_continue_forward()
    
    def _auto_continue_forward(self):
        """自动继续前进0.6秒"""
        if not hasattr(self, '_auto_forward_active') or not self._auto_forward_active:
            return
            
        # 记录开始位置和时间
        self._forward_start_x = self.robot_x
        self._forward_start_y = self.robot_y
        self._forward_start_time = time.time()
        self._forward_step_active = True
        
        # 启动前进步骤定时器
        if not hasattr(self, '_forward_step_timer'):
            self._forward_step_timer = QTimer()
            self._forward_step_timer.timeout.connect(self._check_forward_progress)
        
        self._forward_step_timer.start(50)  # 每50ms检查一次前进进度
        
        # 开始前进
        self.send_car_command(0)
        self.log("🚀 自动继续前进0.6秒...")

    def on_left_button(self):
        """左转按钮 - 修复版本"""
        # 设置移动状态，停止建图
        self.is_moving = True
        self.movement_start_time = time.time()
        
        if self.bluetooth and self.bluetooth.is_connected:
            self.bluetooth.send_car_command(2)  # 2=左转
            self.log("↩️ 发送左转命令")
        elif self.simulator and self.simulator.isRunning():
            self.simulator.turn_left()
            self.log("↩️ 仿真器左转")
        else:
            self.log("⚠️ 请先连接设备或启动仿真器")

    def on_right_button(self):
        """右转按钮 - 修复版本"""
        # 设置移动状态，停止建图
        self.is_moving = True
        self.movement_start_time = time.time()
        
        if self.bluetooth and self.bluetooth.is_connected:
            self.bluetooth.send_car_command(3)  # 3=右转
            self.log("↪️ 发送右转命令")
        elif self.simulator and self.simulator.isRunning():
            self.simulator.turn_right()
            self.log("↪️ 仿真器右转")
        else:
            self.log("⚠️ 请先连接设备或启动仿真器")

    def on_backward_button(self):
        """后退按钮 - 带智能避障检测"""
        self._manual_control_active = True
        self._obstacle_check_timer.start()
        
        # 使用智能控制器
        if self.use_smart_controller:
            self._smart_manual_control('backward')
        else:
            self.send_car_command(4)
            self.log("手动后退（已启用避障检测）")

    def check_obstacle_continuously(self):
        """持续检查障碍物（手动控制模式）"""
        if not self._manual_control_active:
            return
            
        # 检查雷达数据是否可用
        lidar_data_available = False
        if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
            if hasattr(self.radar_viewer, 'lidar_data'):
                lidar_data = self.radar_viewer.lidar_data
                if lidar_data and len(lidar_data) > 0:
                    lidar_data_available = True
                    # 删除频繁的DEBUG输出，提升性能
            else:
                # 雷达数据为空，降低警告频率
                if not hasattr(self, '_empty_lidar_count'):
                    self._empty_lidar_count = 0
                self._empty_lidar_count += 1
                if self._empty_lidar_count % 100 == 0:  # 每100次警告一次
                    self.log("⚠️ 警告：雷达数据为空，避障检测可能失效！")
                    # 删除频繁的DEBUG输出，提升性能
        
        # 优先使用雷达数据检测障碍物
        if lidar_data_available:
            obstacle_detected = self.check_obstacle_from_lidar(lidar_data)
        else:
            # 备用方案：使用地图检测
            obstacle_detected = self.check_obstacle_ahead()
            
        # 真实模式额外检查：如果雷达数据不可用，强制使用地图检测
        if not self.simulation_mode and not lidar_data_available:
            # 删除频繁的DEBUG输出，提升性能
            obstacle_detected = self.check_obstacle_ahead()
            
        if obstacle_detected:
            # 记录检测时间
            import time
            detection_time = time.time()
            if not hasattr(self, '_last_detection_time'):
                self._last_detection_time = detection_time
            else:
                detection_delay = (detection_time - self._last_detection_time) * 1000
                self.log(f"🚨 检测延迟: {detection_delay:.1f}ms")
            
            # 紧急停止：连续发送3次停止命令，确保小车收到
            for i in range(3):
                self.send_car_command(1)  # 紧急停车
            self.log("⚠️ 检测到前方障碍物，紧急停车！")
            self._manual_control_active = False
            self._obstacle_check_timer.stop()
        else:
            # 降低调试输出频率，避免刷屏
            if not hasattr(self, '_check_debug_count'):
                self._check_debug_count = 0
            self._check_debug_count += 1
            # 完全关闭调试信息输出，避免刷屏
            # if self._check_debug_count % 100 == 0:
            #     self.log(f"🔍 避障检测中... 机器人位置: ({self.robot_x:.1f}, {self.robot_y:.1f}), 朝向: {self.robot_theta:.1f}°")

    def on_stop_button(self):
        """停止按钮处理"""
        # 停止智能前进模式
        if hasattr(self, '_smart_forward_active'):
            self._smart_forward_active = False
        
        # 停止前进定时器
        if hasattr(self, '_forward_step_timer'):
            self._forward_step_timer.stop()
        
        # 设置停止状态，恢复建图
        self.is_moving = False
        
        # 发送停止命令
        if self.bluetooth and self.bluetooth.is_connected:
            self.bluetooth.send_car_command(1)  # 1=停止
            self.log("⏹️ 发送停止命令")
        elif self.simulator and self.simulator.isRunning():
            self.simulator.send_command('1')  # 1=停止
            self.log("⏹️ 仿真器停止")
        
        # 清空当前路径
        self.current_path = None
        
        # 停止手动控制避障检测
        self._manual_control_active = False
        self._obstacle_check_timer.stop()
        
        # 停止自动前进模式
        self._auto_forward_active = False
        
        # 停止前进步骤
        if hasattr(self, '_forward_step_active') and self._forward_step_active:
            self._forward_step_active = False
            if hasattr(self, '_forward_step_timer'):
                self._forward_step_timer.stop()
            self.log("⏹️ 停止自动前进模式")
        
        # 停止避障检测等待
        if hasattr(self, '_scan_timer'):
            self._scan_timer.stop()
            self.log("⏹️ 停止避障检测等待")

        # 发送停止命令
        self.send_car_command(1)
        self.log("手动停止")

    def send_car_command(self, cmd):
        """发送小车控制命令"""
        cmd_names = {0: "前进", 1: "停止", 2: "左转", 3: "右转", 4: "后退"}

        if self.simulation_mode:
            # 仿真模式：发送给模拟器
            self.simulator.send_command(str(cmd))
            self.log(f"[SIM] 仿真命令: {cmd_names.get(cmd, '未知')}")
        else:
            # 实物模式：发送给蓝牙
            if self.bluetooth.send_car_command(cmd):
                self.log(f"发送命令: {cmd_names.get(cmd, '未知')}")
            else:
                self.log("发送命令失败（未连接）")

    def open_mapping_window(self):
        """打开建图窗口"""
        try:
            from mapping_window import MappingWindow
            
            # 检查是否已经打开
            if not hasattr(self, 'mapping_window') or self.mapping_window is None:
                self.mapping_window = MappingWindow(
                    map_manager=self.map_manager,
                    path_planner=self.path_planner,
                    parent=self
                )
                
                # 连接信号（删除地图点击和探索相关信号）
                # self.mapping_window.map_clicked.connect(self.on_map_clicked)
                # self.mapping_window.exploration_started.connect(self.start_exploration)
                # self.mapping_window.exploration_stopped.connect(self.stop_exploration)
            
            self.mapping_window.show()
            self.mapping_window.raise_()
            self.mapping_window.activateWindow()
            self.log("建图窗口已打开")
            
        except ImportError as e:
            self.log(f"无法打开建图窗口: {e}")
        except Exception as e:
            self.log(f"打开建图窗口时出错: {e}")
    
    def open_radar_viewer(self):
        """打开雷达查看器"""
        try:
            from radar_viewer import RadarViewer
            
            # 检查是否已经打开
            if not hasattr(self, 'radar_viewer') or self.radar_viewer is None:
                self.radar_viewer = RadarViewer(parent=self)
            
            self.radar_viewer.show()
            self.radar_viewer.raise_()
            self.radar_viewer.activateWindow()
            self.log("雷达查看器已打开")
            
        except ImportError as e:
            self.log(f"无法打开雷达查看器: {e}")
        except Exception as e:
            self.log(f"打开雷达查看器时出错: {e}")

    def start_radar(self):
        """启动雷达 - 优化版本"""
        if self.simulation_mode:
            # 仿真模式：启动模拟雷达
            self.simulator.send_command('7')
            self.log("[SIM] 仿真雷达已启动")
        else:
            # 实物模式：启动真实雷达
            if self.bluetooth.start_radar():
                self.log("雷达已启动")
            else:
                self.log("启动雷达失败")
                return
        
        # 优化：延迟打开窗口，避免阻塞雷达启动
        self.log("准备打开雷达显示器和建图窗口...")
        
        # 使用更长的延迟，确保雷达启动完成
        QTimer.singleShot(500, self._auto_open_windows)  # 延迟500ms

    def _auto_open_windows(self):
        """自动打开窗口的辅助方法"""
        try:
            # 打开雷达查看器
            if not hasattr(self, 'radar_viewer') or self.radar_viewer is None:
                self.open_radar_viewer()
            else:
                self.radar_viewer.show()
                self.radar_viewer.raise_()
                self.radar_viewer.activateWindow()
            
            # 打开建图窗口
            if not hasattr(self, 'mapping_window') or self.mapping_window is None:
                self.open_mapping_window()
            else:
                self.mapping_window.show()
                self.mapping_window.raise_()
                self.mapping_window.activateWindow()
            
            self.log("雷达显示器和建图窗口已自动打开")
        except Exception as e:
            self.log(f"自动打开窗口时出错: {e}")

    def stop_radar(self):
        """停止雷达"""
        if self.simulation_mode:
            # 仿真模式：停止模拟雷达
            self.simulator.send_command('8')
            self.log("[SIM] 仿真雷达已停止")
        else:
            # 实物模式：停止真实雷达
            if self.bluetooth.stop_radar():
                self.log("雷达已停止")
            else:
                self.log("停止雷达失败")
        
        # 可选：停止雷达时最小化窗口（不关闭，方便下次使用）
        if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
            self.radar_viewer.showMinimized()
        if hasattr(self, 'mapping_window') and self.mapping_window is not None:
            self.mapping_window.showMinimized()
        
        self.log("雷达窗口已最小化，可随时重新打开")

    def init_robot_area(self):
        """手动初始化机器人周围区域为空闲"""
        self.map_manager.initialize_robot_area(self.robot_x, self.robot_y, radius=300)
        self.log("手动初始化机器人周围300mm区域为空闲")

    def set_initial_position(self):
        """设置机器人初始位置"""
        try:
            # 获取用户输入的坐标
            new_x = self.input_init_x.value()
            new_y = self.input_init_y.value()
            new_theta = self.input_init_theta.value()
            
            # 更新机器人位置
            self.robot_x = new_x
            self.robot_y = new_y
            self.robot_theta = new_theta
            
            # 更新坐标转换偏移量（用于STM32相对坐标到地图绝对坐标的转换）
            self._coord_offset_x = new_x
            self._coord_offset_y = new_y
            
            # 清空轨迹
            self.trajectory = []
            
            # 更新里程计
            if hasattr(self, 'odometry'):
                self.odometry.reset(new_x / 1000.0, new_y / 1000.0, new_theta)  # 转换为米
            
            # 更新地图管理器中的机器人位置
            if hasattr(self, 'map_manager'):
                # 清空地图（可选）
                # self.map_manager.clear_map()
                pass
            
            # 更新建图窗口中的机器人位置
            if hasattr(self, 'mapping_window') and self.mapping_window is not None:
                if hasattr(self.mapping_window, 'map_visualizer'):
                    self.mapping_window.map_visualizer.set_robot_pose(new_x, new_y, new_theta)
                    # 强制更新显示
                    self.mapping_window.map_visualizer.update()
            
            # 更新雷达查看器中的机器人位置（如果有的话）
            if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
                if hasattr(self.radar_viewer, 'lidar_visualizer'):
                    # 雷达查看器通常不需要更新机器人位置，因为它显示的是相对角度
                    pass
            
            self.log(f"✅ 已设置机器人初始位置: X={new_x}mm, Y={new_y}mm, 朝向={new_theta}°")
            
        except Exception as e:
            self.log(f"❌ 设置初始位置失败: {e}")

    def reset_initial_position(self):
        """重置为默认初始位置"""
        try:
            # 重置为配置文件中的默认值
            self.input_init_x.setValue(config.ROBOT_INIT_X)
            self.input_init_y.setValue(config.ROBOT_INIT_Y)
            self.input_init_theta.setValue(config.ROBOT_INIT_THETA)
            
            # 应用设置
            self.set_initial_position()
            
            self.log("✅ 已重置为默认初始位置")
            
        except Exception as e:
            self.log(f"❌ 重置初始位置失败: {e}")

    # 删除自主探索功能
    # def start_exploration(self):
    #     """开始自主探索"""
    #     import time
    #
    #     # 重置失败计数器
    #     if hasattr(self, '_path_fail_count'):
    #         self._path_fail_count = 0
    #
    #     self.exploration_mode = True
    #     self.exploration_start_time = time.time() * 1000  # 记录开始时间（毫秒）
    #     self.log("开始自主探索模式")
    #
    #     # 初始化机器人周围的区域为空闲，增大初始化范围避免转圈
    #     # 从200mm增加到600mm，确保有足够的空闲区域用于路径规划
    #     self.map_manager.initialize_robot_area(self.robot_x, self.robot_y, radius=600)
    #     self.log("初始化机器人周围600mm区域为空闲")
    #
    #     # 自动启动雷达
    #     self.start_radar()
    #     self.log("等待雷达数据累积（建议等待10秒）...")
    #
    #     # 重置探索相关的标志
    #     if hasattr(self, '_no_frontier_warning_shown'):
    #         delattr(self, '_no_frontier_warning_shown')
    #
    #     self.btn_start_exploration.setEnabled(False)
    #     self.btn_stop_exploration.setEnabled(True)
    #
    # def stop_exploration(self):
    #     """停止探索"""
    #     self.exploration_mode = False
    #     self.send_car_command(1)  # 停车
    #     self.log("停止探索模式")
    #     self.btn_start_exploration.setEnabled(True)
    #     self.btn_stop_exploration.setEnabled(False)

    def return_home(self):
        """返回起点"""
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        home_grid = self.map_manager.world_to_grid(config.ROBOT_INIT_X, config.ROBOT_INIT_Y)

        path = self.path_planner.a_star_safe(robot_grid, home_grid)
        if path:
            smoothed_path = self.path_planner.smooth_path(path)
            self.current_path = smoothed_path
            self.log(f"✅ 规划返回路径: {len(path)}点 → 简化后{len(smoothed_path)}点")
        else:
            self.log("❌ 无法规划返回路径")

    def go_to_coord(self):
        """前往输入的坐标"""
        # 获取输入的坐标
        target_x = self.input_target_x.value()
        target_y = self.input_target_y.value()

        self.log(f"📍 前往坐标: ({target_x}, {target_y})")
        self.log(f"当前位置: ({self.robot_x:.1f}, {self.robot_y:.1f}, {self.robot_theta:.1f}°)")

        # 规划路径
        robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
        target_grid = self.map_manager.world_to_grid(target_x, target_y)

        self.log(f"栅格坐标: 起点{robot_grid} → 终点{target_grid}")

        path = self.path_planner.a_star_safe(robot_grid, target_grid)
        if path:
            smoothed_path = self.path_planner.smooth_path(path)
            self.current_path = smoothed_path
            self.goal_position = (target_x, target_y)
            self.log(f"✅ 规划路径成功: {len(path)}点 → 简化后{len(smoothed_path)}点")
            self.log(f"安全距离: {config.OBSTACLE_EXPANSION * config.CELL_SIZE}mm")
        else:
            self.log("❌ 无法规划路径到该坐标（可能起点或终点在障碍物上）")

    # 删除验收考试相关功能
    # def set_exit_position(self, world_x, world_y):
    #     """设置出口位置（世界坐标）"""
    #     self.exit_position = self.map_manager.world_to_grid(world_x, world_y)
    #     self.log(f"🎯 设置出口位置: ({world_x}, {world_y}) → 栅格({self.exit_position[0]}, {self.exit_position[1]})")
    #
    # def on_set_exit(self):
    #     """设置出口位置（通过点击地图）"""
    #     from PyQt5.QtWidgets import QMessageBox, QInputDialog
    #
    #     # 方式1：输入坐标
    #     world_x, ok1 = QInputDialog.getInt(self, "设置出口", "出口X坐标(mm):",
    #                                        value=3500, min=0, max=5000, step=100)
    #     if not ok1:
    #         return
    #
    #     world_y, ok2 = QInputDialog.getInt(self, "设置出口", "出口Y坐标(mm):",
    #                                        value=3500, min=0, max=5000, step=100)
    #     if not ok2:
    #         return
    #
    #     self.set_exit_position(world_x, world_y)
    #
    #     # 更新独立窗口的出口位置
    #     if hasattr(self, 'mapping_window') and self.mapping_window is not None:
    #         if hasattr(self.mapping_window, 'map_visualizer') and self.mapping_window.map_visualizer is not None:
    #             self.mapping_window.map_visualizer.set_exit_position(self.exit_position)
    #
    #     # 更新状态标签
    #     self.lbl_exit_status.setText(f"出口: ({world_x}, {world_y})")
    #     self.lbl_exit_status.setStyleSheet("color: lime; font-weight: bold;")
    #
    # def go_to_exit(self):
    #     """前往出口"""
    #     if self.exit_position is None:
    #         self.log("⚠️ 未设置出口位置")
    #         return
    #
    #     robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
    #     path = self.path_planner.a_star_safe(robot_grid, self.exit_position)
    #
    #     if path:
    #         smoothed_path = self.path_planner.smooth_path(path)
    #         self.current_path = smoothed_path
    #         self.log(f"✅ 规划到出口路径: {len(path)}点 → 简化后{len(smoothed_path)}点")
    #     else:
    #         self.log("❌ 无法规划到出口的路径")

    def on_data_received(self, data):
        """接收到数据"""
        # 显示在日志中（仅显示非位姿数据，避免刷屏）
        if not data.startswith('X:') and not data.startswith('L:'):
            self.log(f"[数据] {data}")

    def on_pose_updated(self, x, y, theta):
        """位姿更新 - 修复单位转换问题"""
        # 记录原始STM32数据（转换前）
        stm32_x = x
        stm32_y = y

        # 单位转换：STM32发送的是米(m)，Python地图使用毫米(mm)
        # 需要将米转换为毫米：1m = 1000mm
        x_mm = x * 1000  # 米转毫米
        y_mm = y * 1000  # 米转毫米

        # 坐标转换：STM32发送的是相对坐标（从0,0开始），需要加上初始位置偏移
        # 注意：STM32坐标系定义为 - 往前走是x正，左走是y正
        # 需要将STM32的相对坐标转换为地图的绝对坐标
        offset_x = getattr(self, '_coord_offset_x', config.ROBOT_INIT_X)
        offset_y = getattr(self, '_coord_offset_y', config.ROBOT_INIT_Y)
        x = x_mm + offset_x  # STM32相对坐标 + 初始位置偏移
        y = y_mm + offset_y  # STM32相对坐标 + 初始位置偏移

        # 添加调试日志 - 在真实模式下也显示，帮助诊断位姿问题
        if not self.simulation_mode:
            # 真实模式：显示接收到的位姿数据（限制频率避免刷屏）
            if not hasattr(self, '_pose_log_count'):
                self._pose_log_count = 0
                self._last_stm32_pose = (0, 0, 0)

            self._pose_log_count += 1

            # 检测位姿是否有变化
            pose_changed = (abs(stm32_x - self._last_stm32_pose[0]) > 0.01 or  # 0.01m = 10mm
                          abs(stm32_y - self._last_stm32_pose[1]) > 0.01 or
                          abs(theta - self._last_stm32_pose[2]) > 0.5)

            if self._pose_log_count % 10 == 1 or pose_changed:  # 每10次或有变化时显示
                self.log(f"[位姿] STM32原始(m): ({stm32_x:.3f}, {stm32_y:.3f}, {theta:.1f}°)")
                self.log(f"[位姿] 转换后(mm): ({x_mm:.1f}, {y_mm:.1f}, {theta:.1f}°)")
                self.log(f"[位姿] 坐标偏移: ({offset_x:.1f}, {offset_y:.1f})")
                self.log(f"[位姿] 地图坐标(mm): ({x:.1f}, {y:.1f}, {theta:.1f}°)")
                if not pose_changed and self._pose_log_count > 50:
                    self.log("[警告] 位姿长时间未变化，请检查：")
                    self.log("  1. 编码器是否正常工作")
                    self.log("  2. STM32里程计计算是否正确")
                    self.log("  3. 小车是否实际移动")

            self._last_stm32_pose = (stm32_x, stm32_y, theta)

        elif not hasattr(self, '_last_debug_pose'):
            # 仿真模式：首次接收时显示（删除DEBUG输出，提升性能）
            self._last_debug_pose = (x, y, theta)

        # 归一化角度到 0-360 度范围（防止STM32发送的角度累积过大）
        theta = theta % 360.0
        if theta < 0:
            theta += 360.0

        # 更新机器人位姿（这是SLAM的核心！不能删除！）
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

        # 记录轨迹
        self.trajectory.append((x, y))
        if len(self.trajectory) > 1000:
            self.trajectory.pop(0)

        # 更新显示
        self.lbl_pose.setText(f"位置: X={x:.2f} Y={y:.2f}\n朝向: {theta:.1f}°")
        
        # 更新独立窗口的机器人位姿
        if hasattr(self, 'mapping_window') and self.mapping_window is not None:
            self.mapping_window.update_robot_pose(x, y, theta)

    def on_encoder_data_updated(self, left_rev, right_rev, left_speed, right_speed):
        """编码器数据更新"""
        # 调试日志：显示编码器数据接收情况
        if not self.simulation_mode:
            if not hasattr(self, '_encoder_log_count'):
                self._encoder_log_count = 0
                self._last_encoder_data = (0, 0)
                self.log(f"[编码器] 当前里程计模式: {config.ODOMETRY_MODE}")

            self._encoder_log_count += 1

            # 检测编码器是否有变化
            encoder_changed = (abs(left_rev - self._last_encoder_data[0]) > 0.01 or
                             abs(right_rev - self._last_encoder_data[1]) > 0.01)

            if self._encoder_log_count % 10 == 1 or encoder_changed:
                self.log(f"[编码器] L={left_rev:.3f}圈 R={right_rev:.3f}圈 LS={left_speed:.2f} RS={right_speed:.2f}")
                if not encoder_changed and self._encoder_log_count > 50:
                    self.log("[警告] 编码器数据长时间未变化，请检查：")
                    self.log("  1. 编码器是否正确连接")
                    self.log("  2. 电机是否正常转动")
                    self.log("  3. STM32是否正确读取编码器")

            self._last_encoder_data = (left_rev, right_rev)

        # 如果使用编码器模式，更新里程计
        if config.ODOMETRY_MODE == "encoder":
            self.odometry.update_from_encoder(left_rev, right_rev)
            x, y, theta = self.odometry.get_pose()
            # 更新机器人位姿
            self.on_pose_updated(x, y, theta)
        # 如果使用STM32模式，编码器数据应该被忽略
        elif config.ODOMETRY_MODE == "stm32":
            pass  # 不使用编码器数据，等待STM32发送的位姿数据

    def on_lidar_data_updated(self, lidar_data):
        """雷达数据更新 - 移动时停止建图版本"""
        import time

        # 检查是否在移动，移动时停止建图
        if self.is_moving:
            # 只更新独立窗口的雷达数据，不处理地图更新
            if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
                self.radar_viewer.update_lidar_data(lidar_data)
            return

        # 性能优化：限制雷达数据处理频率
        current_time = time.time() * 1000
        if not hasattr(self, '_last_lidar_processing_time'):
            self._last_lidar_processing_time = 0
        
        # 优化：提高雷达数据处理频率，减少延迟
        lidar_processing_interval = 100  # 从200ms减少到100ms，提高响应速度
        if current_time - self._last_lidar_processing_time < lidar_processing_interval:
            # 只更新独立窗口的雷达数据，不处理地图更新
            if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
                self.radar_viewer.update_lidar_data(lidar_data)
            return
        
        self._last_lidar_processing_time = current_time

        # 记录接收到的雷达数据（关闭日志输出，避免刷屏）
        # if self.exploration_mode and len(lidar_data) > 0:
        #     self.log(f"接收雷达数据: {len(lidar_data)} 个点")

        # 旋转时地图更新策略（可配置）
        PAUSE_MAP_DURING_ROTATION = False  # 改为False则旋转时也实时建图

        if PAUSE_MAP_DURING_ROTATION and self.is_rotating:
            # 检查是否旋转超时（超过2秒自动解除旋转状态）
            if current_time - self.last_rotation_command_time > 2000:
                self.is_rotating = False
                self.log("旋转超时，恢复地图更新")
            else:
                # 旋转中，只更新独立窗口的雷达数据，不更新地图
                if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
                    self.radar_viewer.update_lidar_data(lidar_data)
                return

        # 更新地图（优先使用BreezeSLAM）
        if self.use_breezeslam and self.breezeslam_mapper is not None:
            # 使用BreezeSLAM更新地图
            breezeslam_map = self.breezeslam_mapper.update_map(
                self.robot_x, self.robot_y, self.robot_theta, lidar_data
            )
            
            if breezeslam_map is not None:
                # 将BreezeSLAM地图转换为我们的格式
                # BreezeSLAM地图是500x500，我们需要转换为我们的地图大小
                self._update_map_from_breezeslam(breezeslam_map)
            else:
                # BreezeSLAM更新失败，使用传统方法
                self.map_manager.update_from_lidar(
                    self.robot_x, self.robot_y, self.robot_theta, lidar_data
                )
        else:
            # 使用传统建图方法
            self.map_manager.update_from_lidar(
                self.robot_x, self.robot_y, self.robot_theta, lidar_data
            )
        
        # 更新性能统计
        self._performance_stats['lidar_points_processed'] += len(lidar_data)

        # 更新独立窗口的雷达数据
        if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
            self.radar_viewer.update_lidar_data(lidar_data)
        
        # 更新独立窗口的地图数据
        if hasattr(self, 'mapping_window') and self.mapping_window is not None:
            self.mapping_window.update_lidar_data(lidar_data)
        
        self._performance_stats['lidar_updates'] += 1

    def _update_map_from_breezeslam(self, breezeslam_map):
        """
        将BreezeSLAM地图更新到我们的地图管理器
        
        Args:
            breezeslam_map: BreezeSLAM生成的地图 (500x500)
        """
        import numpy as np
        
        # BreezeSLAM地图是500x500，我们的地图是500x500 (5000mm/10mm = 500格)
        # 直接复制地图数据
        if breezeslam_map.shape == self.map_manager.grid_map.shape:
            # 直接复制
            self.map_manager.grid_map = breezeslam_map.copy()
        else:
            # 需要缩放
            try:
                from scipy.ndimage import zoom
                scale_factor = self.map_manager.grid_map.shape[0] / breezeslam_map.shape[0]
                resized_map = zoom(breezeslam_map, scale_factor, order=0)  # 最近邻插值
                self.map_manager.grid_map = resized_map.astype(np.int8)
            except ImportError:
                # 如果没有scipy，使用简单的最近邻缩放
                self._simple_resize_map(breezeslam_map)
    
    def _simple_resize_map(self, source_map):
        """简单的最近邻缩放"""
        import numpy as np
        
        target_shape = self.map_manager.grid_map.shape
        source_shape = source_map.shape
        
        # 计算缩放比例
        scale_y = target_shape[0] / source_shape[0]
        scale_x = target_shape[1] / source_shape[1]
        
        # 创建目标地图
        resized_map = np.zeros(target_shape, dtype=np.int8)
        
        for i in range(target_shape[0]):
            for j in range(target_shape[1]):
                # 计算源地图中的对应位置
                src_i = int(i / scale_y)
                src_j = int(j / scale_x)
                
                # 确保索引在范围内
                src_i = min(src_i, source_shape[0] - 1)
                src_j = min(src_j, source_shape[1] - 1)
                
                resized_map[i, j] = source_map[src_i, src_j]
        
        self.map_manager.grid_map = resized_map

    # 删除地图点击导航功能
    # def on_map_clicked(self, world_x, world_y):
    #     """地图点击事件"""
    #     # 停止探索模式
    #     if self.exploration_mode:
    #         self.stop_exploration()
    #
    #     self.log(f"点击位置: ({world_x:.1f}, {world_y:.1f})")
    #     self.log(f"当前小车位置: ({self.robot_x:.1f}, {self.robot_y:.1f}, {self.robot_theta:.1f}°)")
    #
    #     # 规划路径到点击位置
    #     robot_grid = self.map_manager.world_to_grid(self.robot_x, self.robot_y)
    #     goal_grid = self.map_manager.world_to_grid(world_x, world_y)
    #
    #     self.log(f"栅格坐标: 起点{robot_grid} -> 终点{goal_grid}")
    #
    #     path = self.path_planner.a_star_safe(robot_grid, goal_grid)
    #     if path:
    #         self.current_path = path
    #         self.log(f"规划路径成功，共 {len(path)} 个点（安全距离：{config.OBSTACLE_EXPANSION * config.CELL_SIZE}mm）")
    #         self.goal_position = (world_x, world_y)
    #     else:
    #         self.log("无法规划路径到该位置（可能起点或终点在障碍物上）")

    def update_display(self):
        """定时更新显示 - 优化版本，带性能监控"""
        import time

        # 性能监控：更新计数器
        self._performance_stats['update_count'] += 1
        current_time = time.time() * 1000  # 转换为毫秒

        # 计算FPS（每秒更新次数）
        if current_time - self._performance_stats['last_fps_time'] >= 1000:
            self._performance_stats['fps'] = self._performance_stats['update_count']
            self._performance_stats['update_count'] = 0
            self._performance_stats['last_fps_time'] = current_time
            
            # 每5秒显示一次性能统计
            if self._performance_stats['fps'] > 0 and self._performance_stats['fps'] % 5 == 0:
                self.log(f"[性能] FPS: {self._performance_stats['fps']}, "
                        f"地图更新: {self._performance_stats['map_updates']}, "
                        f"雷达更新: {self._performance_stats['lidar_updates']}, "
                        f"日志更新: {self._performance_stats['log_updates']}, "
                        f"射线计算: {self._performance_stats['ray_calculations']}, "
                        f"雷达点: {self._performance_stats['lidar_points_processed']}")

        # 更新独立窗口的地图显示
        if hasattr(self, 'mapping_window') and self.mapping_window is not None:
            # 建图窗口有自己的更新逻辑，这里不需要额外处理
            pass

        # 如果有路径，执行路径跟踪
        if self.current_path and len(self.current_path) > 0:
            self.path_tracking_control()

    # 删除探索步骤功能
    # def exploration_step(self):
    #     """探索步骤 - 使用优化的前沿探索器"""
    #     # ... 整个函数已删除

    def check_obstacle_from_lidar(self, lidar_data):
        """直接从雷达数据检测前方障碍物
        
        Args:
            lidar_data: 雷达数据列表 [(angle, distance), ...]
            
        Returns:
            True if obstacle detected, False otherwise
        """
        if not lidar_data:
            # 实时警告：没有雷达数据
            self.log("⚠️ 警告：没有雷达数据，无法进行避障检测！")
            return False
            
        obstacle_count = 0
        total_checks = 0
        min_distance = float('inf')  # 记录最小距离
        front_points = []  # 记录前方检测到的点
        
        # 检测前方±45度范围内的障碍物（扩大检测范围）
        for angle, distance in lidar_data:
            # 角度已经是相对于机器人的角度，不需要再减去robot_theta
            # 检查是否在前方±45度范围内
            if abs(angle) <= 45:
                total_checks += 1
                front_points.append((angle, distance))
                # 检查距离是否小于300mm安全距离
                if distance < config.OBSTACLE_SAFETY_DISTANCE:
                    obstacle_count += 1
                    min_distance = min(min_distance, distance)
                    
        # 添加详细的调试信息（降低输出频率避免刷屏）
        if not hasattr(self, '_obstacle_debug_count'):
            self._obstacle_debug_count = 0
        self._obstacle_debug_count += 1
        
        # 大幅减少调试信息输出，避免刷屏
        # 只在检测到障碍物时输出关键信息
        if obstacle_count > 0 and self._obstacle_debug_count % 50 == 0:
            self.log(f"🔍 避障检测: 前方{min_distance:.0f}mm处有障碍物")
        
        # 如果前方有超过0.1%的点检测到障碍物，则认为有障碍物（极低阈值，更敏感）
        if total_checks > 0 and obstacle_count / total_checks > 0.001:
            # 实时输出障碍物警告
            self.log(f"⚠️ 检测到障碍物！前方{min_distance:.0f}mm处有障碍物，障碍物点数: {obstacle_count}/{total_checks}")
            return True
            
        return False

    def check_obstacle_ahead(self, distance_threshold=None):
        """检测前方是否有障碍物（优化版本，更准确）

        Args:
            distance_threshold: 距离阈值（mm），None则使用config.OBSTACLE_CHECK_DISTANCE

        Returns:
            True if obstacle detected, False otherwise
        """
        import math

        if distance_threshold is None:
            distance_threshold = config.OBSTACLE_CHECK_DISTANCE

        obstacle_count = 0
        total_checks = 0
        debug_points = []  # 用于调试的点

        # 优化：检测前方更窄的扇形区域（±20度），避免误判
        for angle_offset in range(-20, 21, 10):  # -20° to +20°, step 10°
            check_angle = self.robot_theta + angle_offset
            rad = math.radians(check_angle)

            # 优化：从更近的距离开始检测，步长更小
            for dist in range(50, int(distance_threshold), 30):  # 从50mm开始，30mm步长
                check_x = self.robot_x + dist * math.cos(rad)
                check_y = self.robot_y + dist * math.sin(rad)

                # 转换为栅格坐标
                grid_x, grid_y = self.map_manager.world_to_grid(check_x, check_y)

                # 检查是否是障碍物
                if self.map_manager.is_valid_grid(grid_x, grid_y):
                    total_checks += 1
                    cell_state = self.map_manager.grid_map[grid_y, grid_x]
                    debug_points.append((grid_x, grid_y, cell_state))
                    
                    if cell_state == config.MAP_STATE_OCCUPIED:
                        obstacle_count += 1

        # 调试信息：只在检测到障碍物时输出详细信息
        if not hasattr(self, '_obstacle_debug_count'):
            self._obstacle_debug_count = 0
        self._obstacle_debug_count += 1
        
        # 完全关闭地图检测调试信息，避免刷屏
        # if obstacle_count > 0 and self._obstacle_debug_count % 10 == 0:
        #     self.log(f"🔍 避障检测详情: 检查了{total_checks}个点，发现{obstacle_count}个障碍物")

        # 优化：提高阈值到40%，减少误判
        if total_checks > 0 and obstacle_count / total_checks > 0.4:
            return True

        return False  # 前方安全

    def path_tracking_control(self):
        """路径跟踪控制 - 走走停停版本，优化雷达建图"""
        import math
        import time

        if not self.current_path or len(self.current_path) < 1:
            return

        # 限制控制频率，避免命令发送过快
        current_time = time.time() * 1000
        if not hasattr(self, 'last_control_time'):
            self.last_control_time = 0

        if current_time - self.last_control_time < config.PATH_TRACKING_INTERVAL:
            return
        self.last_control_time = current_time

        # 检测前方障碍物（实时避障）
        if self.check_obstacle_ahead():
            self.send_car_command(1)  # 紧急停车
            self.log("⚠️ 检测到前方障碍物，紧急停车！")
            # 清空当前路径，等待重新规划
            self.current_path = None
            return

        # 获取下一个目标点
        next_point = self.current_path[0]
        next_world = self.map_manager.grid_to_world(next_point[0], next_point[1])

        # 计算到目标点的距离和方向
        dx = next_world[0] - self.robot_x
        dy = next_world[1] - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # 如果已经到达当前目标点，移除它
        if distance < 80:  # 80mm容差（栅格大小的一半）
            self.current_path.pop(0)
            self.log(f"到达路径点，剩余 {len(self.current_path)} 个点")
            if len(self.current_path) == 0:
                self.send_car_command(1)  # 停车
                self.log("到达目标位置")
                self.current_path = None  # 清空路径
            return

        # 计算目标方向
        target_angle = math.degrees(math.atan2(dy, dx))

        # 计算角度差
        angle_diff = target_angle - self.robot_theta
        # 归一化到[-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # 走走停停导航模式
        if config.NAVIGATION_MODE == "step_by_step":
            self._step_by_step_navigation(angle_diff, distance, current_time)
        else:
            # 连续导航模式（原有逻辑）
            self._continuous_navigation(angle_diff)

    def _step_by_step_navigation(self, angle_diff, distance, current_time):
        """走走停停导航模式"""
        # 初始化导航状态
        if not hasattr(self, '_nav_state'):
            self._nav_state = 'stopped'  # stopped, moving, pausing
            self._nav_start_time = 0
            self._nav_pause_start_time = 0

        # 状态机控制
        if self._nav_state == 'stopped':
            # 停止状态：检查是否需要转向或前进
            if abs(angle_diff) > 10:
                # 需要转向
                self._nav_state = 'turning'
                self._nav_start_time = current_time
                if angle_diff > 0:
                    self.send_car_command(2)  # 左转
                    self.log(f"🔄 开始左转: {angle_diff:.1f}°")
                else:
                    self.send_car_command(3)  # 右转
                    self.log(f"🔄 开始右转: {angle_diff:.1f}°")
            elif distance > config.STEP_MOVE_DISTANCE:
                # 需要前进
                self._nav_state = 'moving'
                self._nav_start_time = current_time
                self.send_car_command(0)  # 前进
                self.log(f"🚀 开始前进: 距离{distance:.1f}mm")
            else:
                # 距离很近，直接到达
                self.current_path.pop(0)
                self.log(f"✅ 到达路径点，剩余 {len(self.current_path)} 个点")

        elif self._nav_state == 'turning':
            # 转向状态：严格按照0.2秒停止
            elapsed = (current_time - self._nav_start_time) / 1000.0
            if elapsed >= config.STEP_MOVE_TIME:
                # 转向完成，停止并进入暂停状态
                self.send_car_command(1)  # 停止
                self._nav_state = 'pausing'
                self._nav_pause_start_time = current_time
                self.log(f"⏸️ 转向完成，暂停{config.STEP_PAUSE_TIME}秒等待雷达建图...")

        elif self._nav_state == 'moving':
            # 前进状态：严格按照0.2秒停止
            elapsed = (current_time - self._nav_start_time) / 1000.0
            if elapsed >= config.STEP_MOVE_TIME:
                # 前进完成，停止并进入暂停状态
                self.send_car_command(1)  # 停止
                self._nav_state = 'pausing'
                self._nav_pause_start_time = current_time
                self.log(f"⏸️ 前进完成，暂停{config.STEP_PAUSE_TIME}秒等待雷达建图...")

        elif self._nav_state == 'pausing':
            # 暂停状态：检查暂停是否完成
            elapsed = (current_time - self._nav_pause_start_time) / 1000.0
            if elapsed >= config.STEP_PAUSE_TIME:
                # 暂停完成，回到停止状态
                self._nav_state = 'stopped'
                self.log("✅ 暂停完成，准备下一步动作")

    def _continuous_navigation(self, angle_diff):
        """连续导航模式（原有逻辑）"""
        # 改进的控制策略（降低旋转阈值，减少原地旋转时间）
        # 1. 如果角度偏差很大（>30度），只转向不前进
        if abs(angle_diff) > 30:
            # 标记旋转状态，暂停地图更新
            self.is_rotating = True
            self.last_rotation_command_time = time.time() * 1000

            if angle_diff > 0:
                self.send_car_command(2)  # 左转
            else:
                self.send_car_command(3)  # 右转
        # 2. 如果角度偏差中等（10-30度），边转边前进
        elif abs(angle_diff) > 10:
            # 小角度调整，不标记为旋转状态
            self.is_rotating = False

            # 先转向，调整到合适角度
            if angle_diff > 0:
                self.send_car_command(2)  # 左转
            else:
                self.send_car_command(3)  # 右转
        # 3. 如果角度偏差小（<10度），直接前进
        else:
            # 解除旋转状态
            self.is_rotating = False
            self.send_car_command(0)  # 前进

    def log(self, message):
        """添加日志 - 优化版本，使用缓冲减少UI更新频率"""
        from datetime import datetime
        import time
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        # 添加到缓冲区
        self._log_buffer.append(log_entry)
        
        # 限制缓冲区大小，避免内存泄漏
        if len(self._log_buffer) > 100:
            self._log_buffer = self._log_buffer[-50:]  # 保留最近50条
        
        # 限制日志更新频率
        current_time = time.time() * 1000
        if current_time - self._last_log_update_time >= config.LOG_UPDATE_INTERVAL:
            self._flush_log_buffer()
            self._last_log_update_time = current_time
            self._performance_stats['log_updates'] += 1
    
    def _flush_log_buffer(self):
        """刷新日志缓冲区到UI"""
        if not self._log_buffer:
            return
        
        # 批量添加日志
        for log_entry in self._log_buffer:
            self.log_text.append(log_entry)
        
        # 清空缓冲区
        self._log_buffer.clear()
        
        # 自动滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
        
        # 限制日志文本长度，避免内存占用过多
        if self.log_text.document().blockCount() > 200:
            # 删除旧的日志行
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.Start)
            for _ in range(50):  # 删除前50行
                cursor.movePosition(cursor.Down, cursor.KeepAnchor)
            cursor.removeSelectedText()
    
    def show_performance_stats(self):
        """显示性能统计信息"""
        stats = self._performance_stats
        self.log("=" * 50)
        self.log("性能统计信息")
        self.log("=" * 50)
        self.log(f"当前FPS: {stats['fps']}")
        self.log(f"地图更新次数: {stats['map_updates']}")
        self.log(f"雷达更新次数: {stats['lidar_updates']}")
        self.log(f"日志更新次数: {stats['log_updates']}")
        self.log(f"射线计算次数: {stats['ray_calculations']}")
        self.log(f"雷达点处理数: {stats['lidar_points_processed']}")
        self.log(f"定时器间隔: {config.UPDATE_INTERVAL}ms")
        self.log(f"地图更新间隔: {config.MAP_UPDATE_INTERVAL}ms")
        self.log(f"雷达更新间隔: {config.LIDAR_UPDATE_INTERVAL}ms")
        self.log(f"日志更新间隔: {config.LOG_UPDATE_INTERVAL}ms")
        self.log(f"探索间隔: {config.EXPLORATION_INTERVAL}ms")
        self.log(f"雷达处理间隔: {config.LIDAR_PROCESSING_INTERVAL}ms")
        self.log(f"最大雷达点数: {config.MAX_LIDAR_POINTS_PER_UPDATE}")
        self.log(f"雷达缓冲区: {config.LIDAR_BUFFER_SIZE}")
        self.log(f"最大射线长度: {config.MAX_RAY_LENGTH}")
        self.log("=" * 50)
        
        # 性能建议
        if stats['fps'] < 2:
            self.log("⚠️ 性能警告：FPS过低，建议检查系统资源")
        elif stats['fps'] > 5:
            self.log("✅ 性能良好：FPS正常")
        else:
            self.log("ℹ️ 性能一般：FPS可接受")
            
    def toggle_controller(self):
        """切换控制器类型"""
        self.use_smart_controller = not self.use_smart_controller
        controller_type = "智能控制器" if self.use_smart_controller else "简单控制器"
        self.log(f"已切换到: {controller_type}")
        self.btn_controller.setText(f"当前: {controller_type}")
        
    def toggle_navigation_mode(self):
        """切换导航模式"""
        if self.btn_nav_mode.isChecked():
            # 切换到走走停停模式
            config.NAVIGATION_MODE = "step_by_step"
            self.btn_nav_mode.setText("走走停停")
            self.log("✅ 已切换到走走停停导航模式")
            self.log(f"   - 每次移动: {config.STEP_MOVE_TIME}秒")
            self.log(f"   - 每次暂停: {config.STEP_PAUSE_TIME}秒")
            self.log(f"   - 移动距离: {config.STEP_MOVE_DISTANCE}mm")
        else:
            # 切换到连续导航模式
            config.NAVIGATION_MODE = "continuous"
            self.btn_nav_mode.setText("连续导航")
            self.log("✅ 已切换到连续导航模式")
            self.log("   - 小车将连续移动，不暂停")
    
    def toggle_slam_mode(self):
        """切换SLAM建图模式"""
        if self.btn_slam_mode.isChecked():
            # 切换到BreezeSLAM模式
            self.use_breezeslam = True
            self.btn_slam_mode.setText("BreezeSLAM")
            self.log("✅ 已切换到BreezeSLAM建图模式")
            self.log("   - 使用高效的SLAM算法")
            self.log("   - 建图速度更快，质量更高")
        else:
            # 切换到传统建图模式
            self.use_breezeslam = False
            self.btn_slam_mode.setText("传统建图")
            self.log("✅ 已切换到传统建图模式")
            self.log("   - 使用射线算法建图")
            self.log("   - 兼容性更好")
        
    def _smart_manual_control(self, direction: str):
        """智能手动控制"""
        # 获取当前位姿
        pose = Pose(self.robot_x, self.robot_y, self.robot_theta)
        
        # 获取雷达数据
        lidar_data = []
        if hasattr(self, 'radar_viewer') and self.radar_viewer is not None:
            if hasattr(self.radar_viewer, 'lidar_data'):
                lidar_data = self.radar_viewer.lidar_data
            
        if lidar_data:
            # 转换雷达数据格式
            angles = []
            ranges = []
            for point in lidar_data:
                if len(point) >= 2:
                    angles.append(point[0])  # 角度（弧度）
                    ranges.append(point[1])  # 距离（mm）
                    
            scan = LidarScan(angles=angles, ranges=ranges)
            
            # 使用智能控制器计算目标位姿
            setpoint = self.manual_controller.compute_setpoint(pose, scan, direction)
            
            if setpoint is None:
                # 需要停车
                self.log("⚠️ 智能控制器检测到障碍物，停止运动")
                self.send_car_command(1)  # 停车
                self._manual_control_active = False
                self._obstacle_check_timer.stop()
            else:
                # 计算运动命令
                dx = setpoint.x - pose.x
                dy = setpoint.y - pose.y
                dtheta = setpoint.theta - pose.theta
                
                # 根据运动类型发送命令
                if abs(dtheta) > 0.1:  # 需要转向
                    if dtheta > 0:
                        self.send_car_command(2)  # 左转
                        self.log(f"🔄 智能左转: {math.degrees(dtheta):.1f}°")
                    else:
                        self.send_car_command(3)  # 右转
                        self.log(f"🔄 智能右转: {math.degrees(dtheta):.1f}°")
                elif abs(dx) > 10 or abs(dy) > 10:  # 需要移动
                    if direction == 'forward':
                        self.send_car_command(0)  # 前进
                        self.log(f"🚀 智能前进: 距离{math.hypot(dx, dy):.1f}mm")
                    elif direction == 'backward':
                        self.send_car_command(4)  # 后退
                        self.log(f"⬅️ 智能后退: 距离{math.hypot(dx, dy):.1f}mm")
                else:
                    self.log("✅ 已到达目标位置")
        else:
            # 没有雷达数据，使用传统方式
            self.log("⚠️ 无雷达数据，使用传统控制")
            if direction == 'forward':
                self.send_car_command(0)
            elif direction == 'backward':
                self.send_car_command(4)
            elif direction == 'left':
                self.send_car_command(2)
            elif direction == 'right':
                self.send_car_command(3)


def main():
    app = QApplication(sys.argv)

    # 设置中文字体
    setup_chinese_font(app)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

