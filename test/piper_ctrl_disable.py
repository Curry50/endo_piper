#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
# 使能机械臂
import time
from piper_sdk import *

# 测试代码
if __name__ == "__main__":
    piper = C_PiperInterface(can_name="can_piper",
                        judge_flag=False,
                        can_auto_init=True,
                        dh_is_offset=1,
                        start_sdk_joint_limit=True,
                        start_sdk_gripper_limit=True,
                        logger_level=LogLevel.WARNING,
                        log_to_file=False,
                        log_file_path=None)
    piper.ConnectPort()
    while(piper.DisablePiper()):
        time.sleep(0.01)
    print("失能成功!!!!")