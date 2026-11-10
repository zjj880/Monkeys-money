import csv
import time
import os
import numpy as np


class DataLogger:
    def __init__(self, worker_port, num_thrusters, dir = 'data'):
        """
        初始化日志记录器
        :param worker_port: 传入 worker 端口号用于区分不同的线程文件
        :param num_thrusters: 推进器数量，用于动态生成 CMDThrust 表头
        """
        # 确保 logs 文件夹存在
        self.log_dir = dir
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # 使用时间戳和端口号作为文件名，确保多线程下文件名唯一
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f"KSK_{worker_port}_{timestamp}.csv")

        # 1. 准备表头
        headers = [
            "Step",
            "FinalTarget_X", "FinalTarget_Y", "FinalTarget_Z",
            "TargetPose_X", "TargetPose_Y", "TargetPose_Z",
            "TargetVelocity_X", "TargetVelocity_Y", "TargetVelocity_Z",
            "Pose_X", "Pose_Y", "Pose_Z",
            "LFPose_X", "LFPose_Y", "LFPose_Z",
            "LFVelocity_X", "LFVelocity_Y", "LFVelocity_Z",
            "Bias_X", "Bias_Y", "Bias_Z",
            "CMDForceLC_X", "CMDForceLC_Y", "CMDForceLC_Z",
            "CMDForceLCPID_X", "CMDForceLCPID_Y", "CMDForceLCPID_Z",
        ]

        # 动态添加推进器表头
        for i in range(num_thrusters):
            headers.append(f"CMDThrust_{i + 1}")

        # 将表头写入 CSV
        with open(self.filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

    def log_step(self, step, final_target, target_pose, target_velocity, pose, lf_pose, lf_velocity, bias, cmd_force, cmd_forcePID, cmd_thrust):
        """
        每次循环调用此函数，将一行数据存入 CSV
        """
        # 将所有数据展平放入一个列表中
        row = [step]

        # 使用 np.array 转换以防传入的是纯列表，并利用 tolist() 展平
        row.extend(np.array(final_target).tolist())
        row.extend(np.array(target_pose).tolist())
        row.extend(np.array(target_velocity).tolist())
        row.extend(np.array(pose).tolist())
        row.extend(np.array(lf_pose).tolist())
        row.extend(np.array(lf_velocity).tolist())
        row.extend(np.array(bias).tolist())
        row.extend(np.array(cmd_force).tolist())
        row.extend(np.array(cmd_forcePID).tolist())
        row.extend(np.array(cmd_thrust).tolist())

        # 2. 追加写入文件
        with open(self.filename, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)