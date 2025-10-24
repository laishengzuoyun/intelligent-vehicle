"""
启动迷宫导航机器人GUI
"""

import sys
from PyQt5.QtWidgets import QApplication
from main_gui import MainWindow, setup_chinese_font


def main():
    """启动GUI"""
    print("=" * 60)
    print("迷宫导航机器人控制系统")
    print("=" * 60)
    print()
    print("[INFO] 启动GUI...")
    
    # 创建应用
    app = QApplication(sys.argv)
    
    # 设置中文字体
    setup_chinese_font(app)
    
    # 创建主窗口
    window = MainWindow()
    window.show()
    
    print("[OK] GUI已启动！")
    print()
    print("[TIP] 使用说明：")
    print("  1. 选择模式：")
    print("     - [实物模式]：连接真实硬件")
    print("     - [仿真模式]：无需硬件，虚拟测试")
    print()
    print("  2. 仿真模式测试：")
    print("     - 点击'[仿真模式]'按钮")
    print("     - 点击'启动雷达'")
    print("     - 点击'前进'/'左转'/'右转'测试控制")
    print("     - 输入目标坐标，点击'前往坐标'进行导航")
    print()
    print("  3. 实物模式测试：")
    print("     - 点击'[实物模式]'按钮")
    print("     - 选择COM口")
    print("     - 点击'连接'")
    print("     - 点击'启动雷达'")
    print("     - 输入目标坐标，点击'前往坐标'进行导航")
    print()
    print("  4. 导航模式说明：")
    print("     - [走走停停]：每次移动0.2秒，暂停2秒，优化雷达建图")
    print("     - [连续导航]：小车连续移动，适合快速导航")
    print("     - 可在导航过程中实时切换模式")
    print("=" * 60)
    
    # 运行应用
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

