from lerobot.common.robot_devices.motors.configs import DynamixelMotorsBusConfig
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
import threading
import time
from typing import Dict
import numpy as np
import serial

class GelloArmController:
    def __init__(self):
        # 初始化Dynamixel舵机
        self.motor_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
        self.motor_index = [1, 2, 3, 4, 5, 6, 7]
        self.motor_model = "xl330-m288"
        self.gripper_model = "xl330-m077"
        self.port = "/dev/ttyUSB0"
        self.motors = {}
        for i in range(len(self.motor_index)):
            self.motors[self.motor_names[i]] = (self.motor_index[i], self.motor_model)
        self.motors[self.motor_names[6]] = (self.motor_index[6], self.gripper_model)
        
        self.config = DynamixelMotorsBusConfig(
        port = self.port,
        motors= self.motors,
    )

        self.motors_bus = DynamixelMotorsBus(self.config)
        self.motors_bus.connect()
        self.motors_bus.write("Torque_Enable", [0,0,0,0,0,0,1])  # 使能舵机
        self.motors_bus.write("Position_P_Gain", [400,400,400,400,400,400,200])
        self.motors_bus.write("Position_D_Gain", [0,0,0,0,0,0,160])
        self.motors_bus.write("Goal_Position", [0,0,0,0,0,0,3072])  # 初始位置

        # 定义关节弧度限制（计算好的范围）
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节 + 1个夹爪
        self.joint_limits = [
            (-60000 / 57324.840764, 60000 / 57324.840764),  # joint1
            (0 / 57324.840764, 120000 / 57324.840764),   # joint2
            (-175000 / 57324.840764, 50000 / 57324.840764),   # joint3
            (-100000 / 57324.840764, 100000 / 57324.840764),  # joint4
            (-75000 / 57324.840764, 75000 / 57324.840764),  # joint5
            (-150000*0 / 57324.840764, 150000*0 / 57324.840764)   # joint6
        ]

        self.joint_factor = 1000/57324.840764
        self.last_joints = None
        self.alpha = 0.95

        # 启动更新线程
        self.running = True
        self.thread = threading.Thread(target=self.update_joint)
        self.thread.start()
    
    def update_joint(self):
        while self.running:
            # 处理事件队列
            try:
                self.leader_joints = self.motors_bus.read("Present_Position")
                self.joints[0] = self.joint_factor*((self.leader_joints[0]-3072)*(180)/2048)
                self.joints[1] = self.joint_factor*((self.leader_joints[1]-2048)*(90)/1024)
                self.joints[2] = self.joint_factor*((self.leader_joints[2]-1024)*(-90)/1024)
                self.joints[3] = self.joint_factor*((self.leader_joints[3]-2048)*(90)/1024)
                self.joints[4] = self.joint_factor*((self.leader_joints[4]-2048)*(-90)/1024)

                # self.joints[6] = self.joint_factor*((self.leader_joints[6]-2048)*(90)/1024)  # 夹爪
                self.joints[6] = self.leader_joints[6]
                
                if self.last_joints is None:
                    self.last_joints = self.joints
                else:
                    # exponential smoothing
                    for i in range(len(self.joints)):
                        self.joints[i] = self.last_joints[i]*(1-self.alpha) + self.joints[i]*self.alpha
                        self.last_joints[i] = self.joints[i]

            except Exception as e:
                self.stop()
                continue
            
            # 末端位置范围保护
            for i in range(len(self.joints)-1):  # 不限制夹爪
                min_val, max_val = self.joint_limits[i]
                self.joints[i] = max(min_val, min(max_val, self.joints[i]))
                self.joints[i] = np.round(self.joints[i], 3)

            if self.joints[6] < 2900:
                self.joints[6] = self.joints[6]/self.joints[6]
            elif self.joints[6] > 3100:
                self.joints[6] = self.joints[6]-self.joints[6]
            else:
                self.joints[6] = self.joints[6]/self.joints[6] + 1  # 保持不动
            
            # 控制更新频率
            time.sleep(0.005)
    
    def get_action(self) -> Dict:
        # 返回机械臂末端的目标状态
        return {
            "joint1": self.joints[0],
            "joint2": self.joints[1],
            "joint3": self.joints[2],
            "joint4": self.joints[3],
            "joint5": self.joints[4],
            "joint6": self.joints[5],
            "gripper": self.joints[6],
        }
    
    def stop(self):
        # 停止更新线程
        self.running = False
        self.thread.join()
        self.motors_bus.disconnect()
        print("gello exits")

    def reset(self):
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节
        # self.joints = [-2.669/57324.840764*1000, 108.282/57324.840764*1000, 
        #                 -100.841/57324.840764*1000, -9.786/57324.840764*1000, 
        #                 65.472/57324.840764*1000, -65.200/57324.840764*1000, 0.0]

# 使用示例
if __name__ == "__main__":
    arm_controller = GelloArmController()
    try:
        while True:
            print(arm_controller.get_action())
            time.sleep(0.1)
    except KeyboardInterrupt:
        arm_controller.stop()