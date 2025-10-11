import time
import serial
import struct
from typing import List, Tuple


class MotorController:
    def __init__(self, port: str, baudrate: int = 115200):
        """
        初始化电机控制器
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.current_zero_position = 0  # 当前0电角度位置

    def send_command(self, command: bytes) -> bytes:
        """
        发送命令并接收响应
        命令格式: AA 01 命令位 00 参数低8位 参数高8位
        """
        self.ser.write(command)
        time.sleep(0.01)
        return self.ser.read_all()

    def set_coarse_zero_position(self):
        """
        使用0B命令拉高IA，得到粗0位
        命令: AA 01 0B 00 00 00
        """
        command = bytes([0xAA, 0x01, 0x0B, 0x00, 0x00, 0x00])
        response = self.send_command(command)
        time.sleep(1)
        _,_,position = self.read_position_data()
        self.set_zero_position(position)
        self.stop_motor ()
        print("设置粗0位完成")
        return response


    def read_position_data(self) -> Tuple[int, int, int]:
        """
        读取位置、速度、电流数据
        命令: AA 01 00 07 00 00
        返回: (位置, 速度, 电流)
        """
        command = bytes([0xAA, 0x01, 0x08, 0x07, 0x00, 0x00])
        response = self.send_command(command)

        if len(response) >= 7:
            # 解析7字节数据: AB + uint16位置 + int16速度 + uint16电流
            # 响应格式: AB [位置低8位] [位置高8位] [速度低8位] [速度高8位] [电流低8位] [电流高8位]
            position = struct.unpack('<H', response[1:3])[0]  # uint16 小端
            position = 0x3fff- position
            speed = struct.unpack('<h', response[3:5])[0]  # int16 小端
            current = struct.unpack('<H', response[5:7])[0]  # uint16 小端
            return position, speed, current
        else:
            print(f"读取数据失败，响应长度: {len(response)}")
            return 0, 0, 0

    def set_zero_position(self, zero_pos: int):
        """
        使用F1命令设定0电角度位置
        命令: AA 01 F1 00 [位置低8位] [位置高8位]
        """
        low_byte = zero_pos & 0xFF
        high_byte = (zero_pos >> 8) & 0xFF
        command = bytes([0xAA, 0x01, 0xF1, 0x00, low_byte, high_byte])
        response = self.send_command(command)
        print(f"设置0电角度位置: {zero_pos} (0x{zero_pos:04X})")
        return response

    def clear_motor_status(self):
        """
        C1命令清除电机状态
        命令: AA 01 C1 00 00 00
        """
        command = bytes([0xAA, 0x01, 0xC0, 0x00, 0x00, 0x00])
        response = self.send_command(command)
        print("清除电机状态完成")
        return response

    def set_motor_speed(self, speed: int):
        """
        C0命令控制电机转速
        命令: AA 01 C0 00 [速度低8位] [速度高8位]
        """
        # 处理负数速度（16位有符号转无符号）
        if speed < 0:
            speed = 0x10000 + speed  # 补码表示

        low_byte = speed & 0xFF
        high_byte = (speed >> 8) & 0xFF

        command = bytes([0xAA, 0x01, 0xC0, 0x00, low_byte, high_byte])
        response = self.send_command(command)

        # 显示实际设置的速度值（有符号）
        actual_speed = speed if speed < 0x8000 else speed - 0x10000
        print(f"设置电机转速: {actual_speed}")
        return response

    def stop_motor(self):
        """
        C0命令停止电机
        命令: AA 01 C0 00 00 00
        """
        command = bytes([0xAA, 0x01, 0xC0, 0x00, 0x00, 0x00])
        response = self.send_command(command)
        print("停止电机")
        return response

    def save_conf(self):
        """
        C0命令停止电机
        命令: AA 01 B0 00 00 00
        """
        command = bytes([0xAA, 0x01, 0xB0, 0x00, 0x00, 0x00])
        response = self.send_command(command)
        print("正在保存设置")
        return response

    def measure_average_speed(self, duration: float = 8.0, frequency: float = 50.0) -> float:
        """
        以指定频率读取电流数据并计算平均值
        """
        interval = 1.0 / frequency
        total_readings = int(duration * frequency)
        speeds = []

        print(f"开始采集电流数据，持续时间: {duration}秒，频率: {frequency}Hz")

        start_time = time.time()
        for i in range(total_readings):
            _, speed, current = self.read_position_data()
            speeds.append(speed)

            # 计算下一次读取的时间
            elapsed = time.time() - start_time
            next_read_time = (i + 1) * interval
            sleep_time = next_read_time - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

        average_speed = sum(speeds) / len(speeds)
        print(f"平均速度: {average_speed:.2f}, 采样点数: {len(speeds)}")
        return average_speed

    def find_zero_electrical_angle(self, initial_zero_pos: int = 0, max_iterations: int = 100) -> int:
        """
        自动寻找0电角度位置
        """
        print("开始自动寻找0电角度位置...")

        # 初始设置
        self.current_zero_position = initial_zero_pos
        self.set_zero_position(self.current_zero_position)

        for iteration in range(max_iterations):
            print(f"\n--- 第 {iteration + 1} 次迭代 ---")
            print(f"当前0电角度位置: {self.current_zero_position}")

            # 清除电机状态
            self.clear_motor_status()
            time.sleep(0.1)

            # 正转测试
            print("正转测试...")
            self.set_motor_speed(25000)
            time.sleep(2)  # 等待电机稳定
            forward_speed = self.measure_average_speed(4.0, 35.0)
            self.stop_motor()
            time.sleep(1)  # 等待电机停止

            # 反转测试
            print("反转测试...")
            self.set_motor_speed(-25000)
            time.sleep(2)  # 等待电机稳定
            reverse_speed = self.measure_average_speed(4.0, 35.0)
            self.stop_motor()
            time.sleep(1)  # 等待电机停止

            # 计算电流差值
            current_difference = abs(forward_speed) - abs(reverse_speed)
            print(f"正转速度: {forward_speed:.2f}, 反转速度: {reverse_speed:.2f}, 差值: {current_difference:.2f}")

            # 判断是否满足条件
            if abs(current_difference) <= 2:
                print(f"\n找到0电角度位置: {self.current_zero_position}")
                print(f"最终速度差值: {current_difference:.2f}")
                return self.current_zero_position

            delta = 2
            # 调整0电角度位置
            if abs(current_difference) > 50:
                delta = 100
            elif abs(current_difference) > 20:
                delta = 25
            elif abs(current_difference) > 10:
                delta = 15
            elif abs(current_difference) > 5:
                delta = 5
            if current_difference > 0:
                # 正转电流大，0电角度位置+10
                self.current_zero_position -= delta
                print("正转速度较大，0电角度位置+10")
            else:
                # 反转电流大，0电角度位置-10
                self.current_zero_position += delta
                print("反转速度较大，0电角度位置-10")

            # 设置新的0电角度位置
            self.set_zero_position(self.current_zero_position)
            time.sleep(0.1)

        print(f"达到最大迭代次数 {max_iterations}，未找到精确的0电角度位置")
        return self.current_zero_position

    def close(self):
        """
        关闭串口连接
        """
        self.ser.close()


def main():
    # 使用示例
    # 请根据实际情况修改串口号
    port = "COM5"  # Windows
    # port = "/dev/ttyUSB0"  # Linux
    # port = "/dev/tty.usbserial"  # macOS

    try:
        # 初始化电机控制器
        motor = MotorController(port)

        # 第一步：设置粗0位
        print("第一步：设置粗0位")
        motor.set_coarse_zero_position()
        time.sleep(1)

        # 读取当前位置
        position, speed, current = motor.read_position_data()
        print(f"当前位置: {position}, 速度: {speed}, 电流: {current}")

        # 第二步：自动寻找0电角度
        print("\n第二步：自动寻找0电角度")
        final_zero_pos = motor.find_zero_electrical_angle(initial_zero_pos=position)

        print(f"\n最终0电角度位置: {final_zero_pos}")

        # 验证最终结果
        print("\n验证最终结果...")
        motor.clear_motor_status()

        # 正转测试
        motor.set_motor_speed(25000)
        time.sleep(2)
        forward_current = motor.measure_average_speed(4.0, 30.0)
        motor.stop_motor()
        time.sleep(2)

        # 反转测试
        motor.set_motor_speed(-25000)
        time.sleep(2)
        reverse_current = motor.measure_average_speed(4.0, 30.0)
        motor.stop_motor()

        final_difference = abs(forward_current) - abs(reverse_current)
        print(
            f"最终验证 - 正转: {forward_current:.2f}, 反转: {reverse_current:.2f}, 差值: {final_difference:.2f}")

        if abs(final_difference) <= 10:
            print("✓ 0电角度设定成功！")
        else:
            print("⚠ 0电角度设定可能不够精确")
        motor.save_conf()
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if 'motor' in locals():
            motor.close()


if __name__ == "__main__":
    main()