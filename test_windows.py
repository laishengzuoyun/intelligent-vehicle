"""
测试独立窗口功能
"""

import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFont

# 导入独立窗口
from mapping_window import MappingWindow
from radar_viewer import RadarViewer

def test_mapping_window():
    """测试建图窗口"""
    app = QApplication(sys.argv)
    
    # 设置中文字体
    font = QFont()
    font.setFamily("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)
    
    window = MappingWindow()
    window.show()
    
    print("建图窗口测试完成")
    return app, window

def test_radar_viewer():
    """测试雷达查看器"""
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
    from PyQt5.QtCore import QTimer
    sim_timer = QTimer()
    sim_timer.timeout.connect(simulate_lidar_data)
    sim_timer.start(1000)
    
    print("雷达查看器测试完成")
    return app, window

def test_both_windows():
    """测试两个窗口同时运行"""
    app = QApplication(sys.argv)
    
    # 设置中文字体
    font = QFont()
    font.setFamily("Microsoft YaHei")
    font.setPointSize(9)
    app.setFont(font)
    
    # 创建建图窗口
    mapping_window = MappingWindow()
    mapping_window.show()
    
    # 创建雷达查看器
    radar_viewer = RadarViewer()
    radar_viewer.show()
    
    # 模拟雷达数据
    import random
    def simulate_lidar_data():
        data = []
        for angle in range(0, 360, 5):
            distance = random.randint(500, 3500)
            data.append((angle, distance))
        radar_viewer.update_lidar_data(data)
        mapping_window.update_lidar_data(data)
    
    # 定时模拟数据
    from PyQt5.QtCore import QTimer
    sim_timer = QTimer()
    sim_timer.timeout.connect(simulate_lidar_data)
    sim_timer.start(1000)
    
    print("两个窗口同时测试完成")
    return app

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == 'mapping':
            app, window = test_mapping_window()
        elif sys.argv[1] == 'radar':
            app, window = test_radar_viewer()
        else:
            app = test_both_windows()
    else:
        app = test_both_windows()
    
    sys.exit(app.exec_())







