import numpy as np


class AdpWaveParaEst:

    def __init__(self):
        # 参数设置
        self.omega_H1 = 2 * np.pi / 300  # 高频截止频率 (rad/s) (y - 低频部分)
        self.omega_L1 = 2 * np.pi / 30  # 低频截止频率 (rad/s)
        self.T_1 = 250  # χ滤波时间常数
        self.omega_H2 = 2 * np.pi / 30  # 高频截止频率 (rad/s) (y - 高频部分)
        self.omega_L2 = 2 * np.pi / 4  # 低频截止频率 (rad/s)
        self.T_2 = 250  # χ滤波时间常数

        # 状态变量
        self.last_y = None  # 对应 nan(3,1)
        self.last_yerr = None  # 对应 nan(3,1)
        self.dt = 1.0  # 时间步长

        # 滤波器1状态 (y - 低频部分)
        self.xi_11 = np.zeros(3)
        self.xi_21 = np.zeros(3)
        self.phi_hat1 = np.zeros(3)
        self.chi1 = np.zeros(3)
        self.A_hat1 = np.zeros(3)

        # 滤波器2状态
        self.xi_12 = np.zeros(3)
        self.xi_22 = np.zeros(3)
        self.phi_hat2 = np.zeros(3)
        self.chi2 = np.zeros(3)
        self.A_hat2 = np.zeros(3)

    def Loop(self, y):
        """
        执行一步参数估计循环。
        参照 AdpWaveParaEst.m (文件 2) 的 Loop 方法。

        Args:
            y (np.ndarray): 输入的 3x1 位移向量 (Pose)。

        Returns:
            tuple: (Ratio, Frq) 两个 3x1 的 numpy 数组。
        """
        # 初始化输出
        Ratio = np.full(3, np.nan)
        Frq = np.full(3, np.nan)

        # 第一次调用时初始化 last_y
        if self.last_y is None:
            self.last_y = y.copy()
            return Ratio, Frq

        # 计算输入 dy 的值 (后向差分)
        dy = (y - self.last_y) / self.dt
        self.last_y = y.copy()  # 更新 last_y

        # 对每个维度独立处理 (i=0, 1, 2)
        for i in range(3):
            # ========== 更新滤波器1 (低频) ==========
            # 估测 A 的幅值 (Chi 滤波)
            self.chi1[i] = (self.T_1 / (self.T_1 + self.dt)) * self.chi1[i] + \
                           (self.dt / (self.T_1 + self.dt)) * (self.xi_11[i] ** 2)
            self.A_hat1[i] = np.sqrt(2 * self.chi1[i])

            # 状态变量导数
            xi_11dot = self.xi_21[i]
            xi_21dot = -(self.omega_H1 + self.omega_L1) * self.xi_21[i] - \
                       (self.omega_H1 * self.omega_L1) * self.xi_11[i] + \
                       self.omega_L1 * dy[i]

            # 更新状态 (欧拉法)
            self.xi_11[i] = self.xi_11[i] + xi_11dot * self.dt
            self.xi_21[i] = self.xi_21[i] + xi_21dot * self.dt

            # ========== 更新滤波器2 (高频) ==========
            # 估测 A 的幅值 (Chi 滤波)
            self.chi2[i] = (self.T_2 / (self.T_2 + self.dt)) * self.chi2[i] + \
                           (self.dt / (self.T_2 + self.dt)) * (self.xi_12[i] ** 2)
            self.A_hat2[i] = np.sqrt(2 * self.chi2[i])

            # 动态更新 k0 (i==0 or i==1 对应 x, y)
            if i == 0 or i == 1:
                k0 = 1.5 / (self.A_hat1[i] ** 2 + 0.2)
            else:
                k0 = 1.5 / (self.A_hat1[i] ** 2 + 0.0001)

            # 状态变量导数
            xi_12dot = self.xi_22[i]
            xi_22dot = -(self.omega_H2 + self.omega_L2) * self.xi_22[i] - \
                       (self.omega_H2 * self.omega_L2) * self.xi_12[i] + \
                       self.omega_L2 * dy[i]

            # 更新状态 (欧拉法)
            self.xi_12[i] = self.xi_12[i] + xi_12dot * self.dt
            self.xi_22[i] = self.xi_22[i] + xi_22dot * self.dt

            # 更新参数估计
            self.phi_hat2[i] = self.phi_hat2[i] + self.dt * k0 * self.xi_12[i] * \
                               (xi_22dot - self.phi_hat2[i] * self.xi_12[i])

            # ========== 计算输出 ==========
            # Ahat1/Ahat2 (LowFrq/HighFrq)，限制最大值为 10
            # 添加 1e-9 避免除零
            Ratio[i] = min(self.A_hat1[i] / (self.A_hat2[i] + 1e-9), 10.0)

            # 高频的 sqrt(abs(phi_hat2))
            Frq[i] = np.sqrt(np.abs(self.phi_hat2[i]))

        return Ratio, Frq

    def Refresh(self):
        """
        重置所有状态变量到初始值。
        参照 AdpWaveParaEst.m (文件 2) 的 Refresh 方法。
        """
        print("AdpWaveParaEst (Python): Refreshing states.")
        # 状态变量
        self.last_y = None

        # 滤波器1状态
        self.xi_11 = np.zeros(3)
        self.xi_21 = np.zeros(3)
        self.phi_hat1 = np.zeros(3)
        self.chi1 = np.zeros(3)
        self.A_hat1 = np.zeros(3)

        # 滤波器2状态
        self.xi_12 = np.zeros(3)
        self.xi_22 = np.zeros(3)
        self.phi_hat2 = np.zeros(3)
        self.chi2 = np.zeros(3)
        self.A_hat2 = np.zeros(3)