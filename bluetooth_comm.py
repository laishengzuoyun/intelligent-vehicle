"""
蓝牙串口通信模块
负责与STM32小车的蓝牙通信
"""

import serial
import serial.tools.list_ports
import threading
import time
from PyQt5.QtCore import QObject, pyqtSignal
import config


class BluetoothComm(QObject):
    """蓝牙通信类"""
    
    # 信号定义
    data_received = pyqtSignal(str)  # 接收到数据
    connection_changed = pyqtSignal(bool)  # 连接状态改变
    pose_updated = pyqtSignal(float, float, float)  # 位姿更新 (x, y, theta)
    lidar_data_updated = pyqtSignal(list)  # 雷达数据更新
    encoder_data_updated = pyqtSignal(float, float, float, float)  # 编码器数据 (L_rev, R_rev, L_speed, R_speed)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.receive_thread = None
        self.lidar_buffer = []  # 雷达数据缓冲区
        self.lidar_scan_complete = False  # 扫描完成标志
        self.running = False
        
    @staticmethod
    def list_ports():
        """列出所有可用串口"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port_name):
        """连接到指定串口"""
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=config.SERIAL_BAUDRATE,
                timeout=config.SERIAL_TIMEOUT
            )
            self.is_connected = True
            self.running = True
            
            # 启动接收线程
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            self.connection_changed.emit(True)
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.connection_changed.emit(False)
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.is_connected = False
        self.connection_changed.emit(False)
    
    def send_command(self, command):
        """发送命令到小车"""
        if not self.is_connected or not self.serial_port:
            return False
        
        try:
            # 添加换行符
            if not command.endswith('\n'):
                command += '\n'
            self.serial_port.write(command.encode('utf-8'))
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def send_car_command(self, cmd_code):
        """发送小车控制命令
        0: 前进, 1: 停止, 2: 左转, 3: 右转, 4: 后退
        """
        return self.send_command(str(cmd_code))
    
    def start_encoder_data(self):
        """开始发送编码器数据"""
        return self.send_command('5')
    
    def stop_encoder_data(self):
        """停止发送编码器数据"""
        return self.send_command('6')
    
    def start_radar(self):
        """启动雷达"""
        return self.send_command('7')
    
    def stop_radar(self):
        """停止雷达"""
        return self.send_command('8')
    
    def get_radar_data(self):
        """获取雷达数据"""
        return self.send_command('R')
    
    def _receive_loop(self):
        """接收数据循环（在独立线程中运行）"""
        buffer = ""
        
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    # 读取数据
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 按行处理数据
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self._parse_data(line)
                            # 过滤雷达数据，不显示在日志中
                            if not line.startswith('Point '):
                                try:
                                    self.data_received.emit(line)
                                except RuntimeError:
                                    # Qt对象已被删除，停止接收
                                    return
                
                time.sleep(0.01)  # 避免CPU占用过高
                
            except Exception as e:
                print(f"接收数据错误: {e}")
                break
    
    def _parse_data(self, line):
        """解析接收到的数据"""
        try:
            # 解析位姿数据: "X:1.23 Y:0.45 T:45.2°"
            if line.startswith('X:'):
                parts = line.split()
                x = float(parts[0].split(':')[1])
                y = float(parts[1].split(':')[1])
                theta_str = parts[2].split(':')[1].replace('°', '')
                theta = float(theta_str)

                # 调试：打印解析结果（已关闭，避免刷屏）
                # if not hasattr(self, '_parse_debug_count'):
                #     self._parse_debug_count = 0
                # if self._parse_debug_count < 3:
                #     print(f"[蓝牙解析] 原始数据: {line}")
                #     print(f"[蓝牙解析] 解析结果: X={x} Y={y} θ={theta}")
                #     self._parse_debug_count += 1

                try:
                    self.pose_updated.emit(x, y, theta)
                except RuntimeError:
                    # Qt对象已被删除，停止解析
                    return
            
            # 解析编码器数据: "L:12.34,R:56.78,LS:0.30,RS:0.31"
            elif line.startswith('L:'):
                parts = line.split(',')
                l_rev = float(parts[0].split(':')[1])
                r_rev = float(parts[1].split(':')[1])
                l_speed = float(parts[2].split(':')[1])
                r_speed = float(parts[3].split(':')[1])
                
                try:
                    self.encoder_data_updated.emit(l_rev, r_rev, l_speed, r_speed)
                except RuntimeError:
                    # Qt对象已被删除，停止解析
                    return
            
            # 解析雷达数据: "Point 0: A=45.5° D=1234mm Q=85"
            elif line.startswith('Point'):
                try:
                    parts = line.split()
                    # 检查数据完整性
                    if len(parts) >= 4:
                        # 提取角度和距离
                        angle_str = parts[2].replace('A=', '').replace('°', '').strip()
                        dist_str = parts[3].replace('D=', '').replace('mm', '').strip()

                        # 验证数据有效性
                        if angle_str and dist_str:
                            angle = float(angle_str)
                            distance = int(dist_str)

                            # 过滤无效数据
                            if 0 <= angle <= 360 and 0 < distance < 6000:
                                # 累积雷达点
                                self.lidar_buffer.append((angle, distance))

                                # 性能优化：累积更多点再发送，减少射线计算频率
                                if len(self.lidar_buffer) >= config.LIDAR_BUFFER_SIZE:
                                    # 发送完整的扫描数据
                                    # 删除频繁的DEBUG输出，提升性能
                                    try:
                                        self.lidar_data_updated.emit(self.lidar_buffer.copy())
                                        self.lidar_buffer.clear()
                                    except RuntimeError:
                                        # Qt对象已被删除，停止解析
                                        return
                except (ValueError, IndexError):
                    # 数据格式错误，忽略
                    pass
                    
        except Exception as e:
            # 解析失败，忽略该行
            pass
    
    def set_pid_params(self, left_kp, left_ki, left_kd, right_kp, right_ki, right_kd):
        """设置PID参数"""
        self.send_command(f"PID_L_{left_kp},{left_ki},{left_kd}")
        time.sleep(0.05)
        self.send_command(f"PID_R_{right_kp},{right_ki},{right_kd}")
    
    def set_target_speed(self, left_speed, right_speed):
        """设置目标速度"""
        return self.send_command(f"SPEED_{left_speed},{right_speed}")

