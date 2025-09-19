import pygame
import threading
import time
from typing import Dict

class SixAxisArmController:
    def __init__(self):
        # 初始化pygame和手柄
        pygame.init()
        pygame.joystick.init()
        
        # 检查是否有连接的手柄
        if pygame.joystick.get_count() == 0:
            raise Exception("未检测到手柄")
        
        # 初始化手柄
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # 初始化关节和夹爪状态
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 机械臂末端位置 [X, Y, Z, RX, RY, RZ]
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节
        self.gripper = 0.0  # 夹爪状态
        self.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 机械臂末端的速度
        self.gripper_speed = 0.0  # 夹爪速度
        
        # 定义关节弧度限制（计算好的范围）
        # self.joint_limits = [
        #     (-150000 / 57324.840764, 150000 / 57324.840764),  # joint1
        #     (0 / 57324.840764, 195000 / 57324.840764),   # joint2
        #     (-175000 / 57324.840764, 0 / 57324.840764),   # joint3
        #     (-100000 / 57324.840764, 100000 / 57324.840764),  # joint4
        #     (-75000 / 57324.840764, 75000 / 57324.840764),  # joint5
        #     (-150000 / 57324.840764, 150000 / 57324.840764)   # joint6
        # ]

        self.position_limits = [
            (40.0, 600.0),   # X轴范围
            (-100.0, 380.0),  # Y轴范围
            (200.0, 400.0),   # Z轴范围
            (-170.0, 170.0),    # RX轴范围
            (-60.0, 90.0),    # RY轴范围
            (-160.0, 160.0)     # RZ轴范围
        ]

        # 启动更新线程
        self.running = True
        self.thread = threading.Thread(target=self.update_position)
        self.thread.start()
    
    def update_position(self):
        while self.running:
            # 处理事件队列
            try:
                pygame.event.pump()
            except Exception as e:
                self.stop()
                continue
                
            # 获取摇杆和按钮输入
            left_x = self.joystick.get_axis(0)  # 左摇杆x轴
            if abs(left_x) < 0.5:
                left_x = 0.0

            left_y = -self.joystick.get_axis(1)  # 左摇杆y轴（取反，因为y轴向下为正
            if abs(left_y) < 0.5:
                left_y = 0.0

            right_x = self.joystick.get_axis(3)  # 右摇杆x轴（取反，因为y轴向下为正）
            if abs(right_x) < 0.5:
                right_x = 0.0
            
            right_y = -self.joystick.get_axis(4)  # 右摇杆y轴（取反，因为y轴向下为正）
            if abs(right_y) < 0.5:
                right_y = 0.0
            
            # 获取方向键输入
            hat = self.joystick.get_hat(0)
            up = hat[1] == 1
            down = hat[1] == -1
            left = hat[0] == -1
            right = hat[0] == 1
            
            # 获取按钮输入
            circle = self.joystick.get_button(1)  # 圈按钮
            cross = self.joystick.get_button(0)  # 叉按钮
            triangle = self.joystick.get_button(2)
            square = self.joystick.get_button(3)
            
            # 映射输入到速度
            self.speeds[0] = left_x * 0.3
            self.speeds[1] = left_y * 0.3
            self.speeds[2] = right_y * 0.3
            self.speeds[3] = 0.1 if circle else (-0.1 if cross else 0.0)
            self.speeds[4] = 0.4 if up else (-0.4 if down else 0.0)
            self.speeds[5] = 0.6 if right else (-0.6 if left else 0.0)
            # self.gripper_speed = 0.1 if circle else (-0.1 if cross else 0.0)
            
            # 积分速度到末端位置
            for i in range(6):
                # self.joints[i] += self.speeds[i]
                self.position[i] += self.speeds[i]
            # self.gripper += self.gripper_speed
            
            # 末端位置范围保护
            for i in range(6):
                min_val, max_val = self.position_limits[i]
                self.position[i] = max(min_val, min(max_val, self.position[i]))
            
            # 夹爪范围保护（0~0.08弧度）
            # self.gripper = max(0.0, min(0.08, self.gripper))
            
            # 控制更新频率
            time.sleep(0.02)
    
    def get_action(self) -> Dict:
        # 返回机械臂末端的目标状态
        return {
            "X": self.position[0],
            "Y": self.position[1],
            "Z": self.position[2],
            "RX": self.position[3],
            "RY": self.position[4],
            "RZ": self.position[5],
        }
    
    def stop(self):
        # 停止更新线程
        self.running = False
        self.thread.join()
        pygame.quit()
        print("Gamepad exits")

    def reset(self):
        # self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节
        # self.joints = [-2.669/57324.840764*1000, 108.282/57324.840764*1000, 
        #                 -100.841/57324.840764*1000, -9.786/57324.840764*1000, 
        #                 65.472/57324.840764*1000, -65.200/57324.840764*1000, 0.0] # [6 joints + 1 gripper] * 0.0
        self.position = [409.855,-33.193,
                         307.207,-155.978,
                         0.058,-115.125]  # 机械臂末端位置 [X, Y, Z, RX, RY, RZ]
        self.gripper = 0.0  # 夹爪状态
        self.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节的速度
        self.gripper_speed = 0.0  # 夹爪速度

# 使用示例
if __name__ == "__main__":
    arm_controller = SixAxisArmController()
    try:
        while True:
            print(arm_controller.get_action())
            time.sleep(0.1)
    except KeyboardInterrupt:
        arm_controller.stop()