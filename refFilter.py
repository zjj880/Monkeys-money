import numpy as np
import math
import para_def as para


class REFFilter:
    def __init__(self, initial_target_pose):

        # 系统参数
        self.Delta = para.RefDelta
        self.Omega = para.RefOmega
        self.MaxV = para.RefMaxV
        self.I = np.eye(3, dtype=np.float64)

        # 采样
        self.Ts = 1.0
        self.first_time = True

        # 计算系统动力学矩阵
        self.A = np.linalg.matrix_power(self.Omega, 3)
        self.B = (2 * self.Delta + self.I) @ np.linalg.matrix_power(self.Omega, 2)
        self.C = (2 * self.Delta + self.I) @ self.Omega

        # 状态变量初始化
        # eta_d: 当前滤波器输出的期望位置; eta_d_dot: 期望速度
        self.eta_d = np.array(initial_target_pose, dtype=np.float64).flatten()
        self.eta_d_dot = np.zeros(3, dtype=np.float64)
        self.eta_d_ddot = np.zeros(3, dtype=np.float64)
        self.eta_d_d3 = np.zeros(3, dtype=np.float64)

        # 最终目标点 r
        self.r = np.array(initial_target_pose, dtype=np.float64).flatten()

    def update_final_target(self, new_r):
        # 更新目标位置
        self.r = np.array(new_r, dtype=np.float64).flatten()

    def reset_state(self, current_pose, current_vel=np.zeros(3)):
        # 定当前点
        self.eta_d = np.array(current_pose, dtype=np.float64).flatten()
        self.eta_d_dot = np.array(current_vel, dtype=np.float64).flatten()
        self.eta_d_ddot = np.zeros(3, dtype=np.float64)
        self.eta_d_d3 = np.zeros(3, dtype=np.float64)
        self.r = self.eta_d.copy()

    def Loop(self, current_pose, current_vel):
        if self.first_time:
            self.first_time = False
            # 第一次运行时，实际目标点从船舶当前位置开始，保证轨迹平滑无突变
            self.eta_d = np.array(current_pose, dtype=np.float64).flatten()
            self.eta_d_dot = np.array(current_vel, dtype=np.float64).flatten()
            self.eta_d_ddot = np.zeros(3, dtype=np.float64)
            self.eta_d_d3 = np.zeros(3, dtype=np.float64)

            # 第一次迭代直接返回当前状态，不进行积分
            return self.eta_d.copy(), self.eta_d_dot.copy()

        # 三阶系统动态方程求解
        self.eta_d_d3 = (self.A @ (self.r - self.eta_d)
                         - self.B @ self.eta_d_dot
                         - self.C @ self.eta_d_ddot)

        # 欧拉法离散化积分
        self.eta_d_ddot += self.eta_d_d3 * self.Ts
        self.eta_d_dot += self.eta_d_ddot * self.Ts

        # 速度限幅处理
        for i in range(3):
            if abs(self.eta_d_dot[i]) >= self.MaxV[i]:
                self.eta_d_dot[i] = np.sign(self.eta_d_dot[i]) * self.MaxV[i]

        # 位置积分
        self.eta_d += self.eta_d_dot * self.Ts

        return self.eta_d.copy(), self.eta_d_dot.copy()