import numpy as np
import para_def as para


class PID2:
    def __init__(self):
        # 增益
        self.Kp = para.Kp
        self.Ki = para.Ki
        self.Kd = para.Kd
        # 限幅
        self.FLim = para.FLim

        # 状态变量
        self.integral = np.zeros(3)  # 积分项
        self.firsttime = True  # 首次调用标志
        self.dt = 1.0

    def SetPara(self, controlLevel):
        Level = np.array([0.0 if x == 0 else 0.5 if x == 1 else 1.5 if x == 3 else 1.0
                          for x in controlLevel], dtype=np.float64)

        self.Kd, self.Kp, self.Ki = self.Kd * Level, self.Kp * Level, self.Ki * Level



    def ClearIntegral(self):
        """
        清零积分项。
        对应 MATLAB 中的 ClearIntegral。
        """
        self.integral = np.zeros(3)
        print("PID2: Integral term cleared.")

    def Loop(self, target_pos, target_vel, current_pos, current_vel, bias):     # np.ndarray: 计算出的三自由度控制力/力矩 CMDForce (3x1, 单位 kN)。

        # 1. 计算误差 积分项 微分项 积分项 (Global)
        error = target_pos - current_pos
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        self.integral = self.integral + error
        derivative = target_vel - current_vel

        # 2. 构建旋转矩阵 J (基于当前位置 current_pos[2] 即 Heading/Psi)
        psi = current_pos[2]
        J = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        # 3. 计算误差 积分项 微分项 积分项 和 bias (Local)
        errorLC = J.T @ error
        integralLC = J.T @ self.integral
        derivativeLC = J.T @ derivative
        biasLC = J.T @ bias

        # 4. 计算 PID 力矩 (Local)
        P_term = np.multiply(self.Kp, errorLC)
        I_term = np.multiply(self.Ki, integralLC)
        D_term = np.multiply(self.Kd, derivativeLC)

        # 5. 核心计算: F_Global = J' * (P + I + D - biasLC)
        F_local = P_term + I_term + D_term - biasLC
        for i in range(3):
            F_local[i] = np.clip(F_local[i], -self.FLim[i], self.FLim[i])
            if self.Kp[i] == 0:
                F_local[i] = 0
        F_global = J @ F_local

        return F_global, F_local