"""
配置文件 - 迷宫导航机器人GUI系统
"""

# ==================== 串口通信配置 ====================
BAUD_RATE = 921600        # 波特率（与STM32的USART1一致）
SERIAL_BAUDRATE = 921600  # 波特率（与STM32的USART1一致）
SERIAL_TIMEOUT = 0.1      # 串口超时时间（秒）

# ==================== 地图配置 ====================
# 验收考试配置：280cm × 280cm 实际场地
# 为了避免机器人移动时坐标变负，地图设置为 5000mm × 5000mm
# 机器人初始位置在地图中心 (2500, 2500)，这样向任何方向移动都不会越界
# 注意：STM32发送的位姿单位是米(m)，Python地图单位是毫米(mm)
MAP_WIDTH = 5000          # 世界坐标宽度（mm）- 扩大到5m×5m
MAP_HEIGHT = 5000         # 世界坐标高度（mm）
WORLD_WIDTH = 5000        # 世界坐标宽度（mm）
WORLD_HEIGHT = 5000       # 世界坐标高度（mm）
CELL_SIZE = 10            # 栅格地图单元格大小（mm）
SCALE_FACTOR = 40         # JSON地图缩放因子

# 机器人初始位置（地图中心）
ROBOT_INIT_X = 2500       # 初始X坐标（mm）- 地图中心
ROBOT_INIT_Y = 2500       # 初始Y坐标（mm）- 地图中心
ROBOT_INIT_THETA = 0      # 初始朝向（度）

# 地图状态
MAP_STATE_UNKNOWN = -1    # 未知区域
MAP_STATE_FREE = 0        # 空闲区域
MAP_STATE_OCCUPIED = 1    # 障碍物
MAP_STATE_ENCLOSED = 2    # 封闭区域

# ==================== 颜色配置 ====================
COLOR_BACKGROUND = (128, 128, 128)   # 灰色背景
COLOR_WALL = (64, 64, 64)            # 深灰色障碍物
COLOR_FREE = (255, 255, 255)         # 白色已探索区域
COLOR_ROBOT = (0, 200, 255)          # 天蓝色机器人
COLOR_PATH = (255, 100, 100)         # 浅红色路径
COLOR_LIDAR = (255, 255, 0)          # 黄色雷达点
COLOR_TRAJECTORY = (255, 0, 0)       # 红色轨迹
COLOR_EXIT = (0, 255, 0)             # 绿色出口

# ==================== 机器人参数 ====================
ROBOT_RADIUS = 10         # 机器人半径（mm）
WHEEL_BASE = 150          # 轮距（mm）
WHEEL_DIAMETER = 67       # 轮子直径（mm）
ENCODER_RESOLUTION = 1000 # 编码器分辨率（脉冲/圈）

# 里程计模式
ODOMETRY_MODE = "stm32"   # "stm32": 使用STM32计算的位姿, "encoder": 使用编码器计算

# ==================== 雷达参数 ====================
LIDAR_MAX_RANGE = 6000    # 雷达最大量程（mm）- 优化：从5000增加到6000
LIDAR_ANGLE_RESOLUTION = 5  # 雷达角度分辨率（度）

# ==================== 控制参数 ====================
UPDATE_INTERVAL = 50        # GUI更新间隔（ms）- 进一步降低到50ms，提高响应速度
EXPLORATION_INTERVAL = 2000 # 探索步骤间隔（ms）- 进一步降低探索频率
DATA_BUFFER_SIZE = 1000    # 数据缓冲区大小
MAP_UPDATE_INTERVAL = 500   # 地图可视化更新间隔（ms）- 大幅降低到500ms
LIDAR_UPDATE_INTERVAL = 500  # 雷达可视化更新间隔（ms）- 大幅降低到500ms
LOG_UPDATE_INTERVAL = 1000  # 日志更新间隔（ms）- 进一步降低日志更新频率

# ==================== 射线算法优化参数 ====================
LIDAR_PROCESSING_INTERVAL = 100  # 雷达数据处理间隔（ms）- 大幅降低到100ms，提高响应速度
MAX_LIDAR_POINTS_PER_UPDATE = 30  # 每次处理的最大雷达点数 - 增加到30个点，提高建图效率
MAX_RAY_LENGTH = 200  # 最大射线长度（栅格单位）
LIDAR_BUFFER_SIZE = 20  # 雷达数据缓冲区大小 - 进一步减少缓冲区，降低内存占用

# ==================== 路径规划参数 ====================
# 通道宽度700mm，小车宽度约200mm，安全距离设置为80mm（8格）
# 所需通道宽度 = 200 + 80*2 = 360mm < 700mm，可以安全通过
OBSTACLE_EXPANSION = 8    # 障碍物膨胀半径（格子数）- 8格=80mm，适配700mm通道
PATH_SMOOTHING = True     # 是否启用路径平滑
OBSTACLE_CHECK_DISTANCE = 800  # 前方障碍物检测距离（mm）- 增加到80cm更安全
OBSTACLE_SAFETY_DISTANCE = 400  # 避障安全距离（mm）- 调整为40cm

# ==================== 导航控制参数 ====================
NAVIGATION_MODE = "step_by_step"  # 导航模式: "continuous"连续导航, "step_by_step"走走停停
STEP_MOVE_TIME = 0.2  # 每次移动时间（秒）
STEP_PAUSE_TIME = 2.0  # 每次暂停时间（秒）
STEP_MOVE_DISTANCE = 30  # 每次移动的最大距离（mm）
PATH_TRACKING_INTERVAL = 200  # 路径跟踪控制间隔（ms）

# ==================== JSON地图路径 ====================
# 默认地图文件路径（可在GUI中修改）
DEFAULT_MAP_PATH = "../submission/1.json"

# ==================== 验收考试配置 ====================
EXAM_MODE = False         # 是否启用验收模式
EXAM_GRID_SIZE = 4        # 4×4方格
EXAM_CELL_SIZE = 700      # 每个单元格70cm = 700mm

# 入口和出口坐标（验收前一天公布后填写）
ENTRANCE_CELL = (0, 0)    # 入口单元格坐标
EXIT_CELL = (3, 2)        # 出口单元格坐标（示例，实际以公布为准）

