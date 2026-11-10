import numpy as np
from adp_wave_para_est import AdpWaveParaEst


# ==============================================================================
# === 辅助内部类 (用于管理参数) ===
# ==============================================================================

class _TuningParams:
    """
    用于存储 NPOV4.m (文件 1) 中 TP 结构体的所有参数
    """

    def __init__(self):
        self.gamma = 1.0
        self.zetax = 0.08
        self.zetay = 0.08
        self.zetah = 0.08
        self.zetanx = 1.2
        self.zetany = 1.2
        self.zetanh = 1.2
        self.omegax = 2 * np.pi / 10
        self.omegay = 2 * np.pi / 10
        self.omegah = 2 * np.pi / 10
        self.omegac = 1.2
        self.bias = 20000.0
        self.lambda_coeffx = 0.2
        self.lambda_coeffy = 0.2
        self.lambda_coeffh = 0.2
        self.k_til1 = 10 ** 3 * 600
        self.k_til2 = 10 ** 3 * 900
        self.k_til3 = 10 ** 3 * 500000
        self.dt = 1.0


class _VesselModel:
    """
    用于存储 NPOV4.m (文件 1) 中 VM 结构体的 M 和 D 矩阵
    """

    def __init__(self):
        self.M = np.eye(3)
        self.D = np.eye(3)


# ==============================================================================
# === NPOV4 Python 实现 ===
# ==============================================================================

class NPOV4:
    def __init__(self):
        # 参数
        self.TP = _TuningParams()
        self.VM = _VesselModel()

        # 依赖
        self.adpWave = AdpWaveParaEst()

        # 设置为 'Auto' 以确保 AdpWaveParaEst (文件 2) 被调用
        self.mode = 'Manual'

        # 内部状态变量 (对应 NPOV4.m properties)
        self.y_mea = np.zeros(3)
        self.y_est = np.zeros(3)
        self.y_err = np.zeros(3)
        self.y_low = np.zeros(3)
        self.y_low_l = np.zeros(3)
        self.y_wave = np.zeros(3)
        self.v_est = np.zeros(3)
        self.v_est_l = np.zeros(3)
        self.xi_est = np.zeros(6)
        self.xi_est_l = np.zeros(6)
        self.b_est = np.zeros(3)
        self.b_est_l = np.zeros(3)
        self.b_jump = np.zeros(3)
        self.tau = np.zeros(3)
        self.tmp = np.zeros(3)

        # 内部矩阵
        self.Omega = np.zeros((6, 6))
        self.K1 = np.zeros((6, 3))
        self.K2 = np.zeros((3, 3))
        self.J = np.eye(3)
        self.JLow = np.eye(3)
        self.T = 2000 * np.eye(3)
        self.Lambda = np.zeros((3, 3))
        self.varGamma = np.zeros((3, 6))
        self.Ktil = np.zeros((3, 3))

        # 状态标志
        self.firsttime = True

        # 替代 MATLAB persistent 变量
        self.lastmodeStr = 'Manual'
        self.adp_step = 0

        # 初始化
        self.UpdateMatrices()

    def UpdateMatrices(self):
        # Omega matrix
        Omega21 = np.diag([-self.TP.omegax ** 2, -self.TP.omegay ** 2, -self.TP.omegah ** 2])
        Omega22 = np.array([
            [-2 * self.TP.zetax * self.TP.omegax, 0, 0],
            [0, -2 * self.TP.zetay * self.TP.omegay, 0],
            [0, 0, -2 * self.TP.zetah * self.TP.omegah]
        ])
        self.Omega = np.block([
            [np.zeros((3, 3)), np.eye(3)],
            [Omega21, Omega22]
        ])

        # K1 matrix
        k_11 = -2 * self.TP.omegac * (self.TP.zetanx - self.TP.zetax) / self.TP.omegax
        k_12 = -2 * self.TP.omegac * (self.TP.zetany - self.TP.zetay) / self.TP.omegay
        k_13 = -2 * self.TP.omegac * (self.TP.zetanh - self.TP.zetah) / self.TP.omegah
        k_21 = 2 * self.TP.omegax * (self.TP.zetanx - self.TP.zetax)
        k_22 = 2 * self.TP.omegay * (self.TP.zetany - self.TP.zetay)
        k_23 = 2 * self.TP.omegah * (self.TP.zetanh - self.TP.zetah)
        self.K1 = np.array([
            [k_11, 0, 0], [0, k_12, 0], [0, 0, k_13],
            [k_21, 0, 0], [0, k_22, 0], [0, 0, k_23]
        ])

        # Other matrices
        self.K2 = self.TP.omegac * np.eye(3)
        self.Ktil = np.diag([self.TP.k_til1, self.TP.k_til2, self.TP.k_til3])
        self.Lambda = np.diag([
            self.TP.lambda_coeffx * self.TP.k_til1,
            self.TP.lambda_coeffy * self.TP.k_til2,
            self.TP.lambda_coeffh * self.TP.k_til3
        ])
        self.varGamma = np.block([[np.zeros((3, 3)), np.eye(3)]])
        self.T = np.array([
            [self.TP.bias, 0, 0],
            [0, self.TP.bias, 0],
            [0, 0, 10 * self.TP.bias]
        ])

    # ==========================================================================
    # === 公共接口方法 (供 TCP_Worker 调用) ===
    # ==========================================================================

    def SetPara(self, M, D):
        self.VM.M = M
        self.VM.D = D
        self.UpdateMatrices()

    def NPOLoop(self, y_mea, thrust):
        # 1. 刷新变量
        self.RefreshVariables(y_mea, thrust)

        # 2. 自适应率
        modeStr = self.mode  # 使用类属性替代 GUI

        if modeStr != self.lastmodeStr:
            self.adpWave.Refresh()
            self.b_jump = np.zeros(3)
            self.adp_step = 0

        if modeStr == 'Auto':
            self.adp_step += 1
            RATIO, FRQ = self.adpWave.Loop(y_mea)

            if not np.any(np.isnan(FRQ[0:3])):
                self.TP.omegax = min(1.1, max(FRQ[0], 0.35))
                self.TP.zetanx = -0.667 * self.TP.omegax + 1.83
                self.TP.omegay = min(1.1, max(FRQ[1], 0.35))
                self.TP.zetany = -0.667 * self.TP.omegay + 1.83
                self.TP.omegah = min(1.1, max(FRQ[2], 0.35))
                self.TP.zetanh = -0.667 * self.TP.omegah + 1.83
                self.JumpEstimator(RATIO)

            self.UpdateMatrices()

        self.lastmodeStr = modeStr

        # 3. 估算器
        self.WaveEstimator()
        self.BiasEstimator()
        self.VelocityEstimator()
        self.LowEstimator()

        # 4. 更新估计
        self.y_est = self.y_low + self.y_wave
        self.y_err = self.y_mea - self.y_est
        self.y_err[2] = np.arctan2(np.sin(self.y_err[2]), np.cos(self.y_err[2]))

    def GetLFMotion(self, t):
        LFPosition = self.y_low.copy()

        # 更新旋转矩阵 J
        psi = self.y_mea[2]
        self.J = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        LFVelocity = self.J @ self.v_est

        # 转换为 kN (匹配 tcp_worker 的单位)
        b = (self.b_est / 1000.0) + (self.b_jump / 1000.0)
        # (MATLAB 中的所有绘图代码 已在此处移除)

        return LFPosition, LFVelocity, b

    # ==========================================================================
    # === 内部私有方法 (NPOV4 内部逻辑) ===
    # ==========================================================================

    def RefreshVariables(self, y_mea, thrust):
        """ 参照 MATLAB NPOV4.m (文件 1) 中的 RefreshVariables """
        self.y_mea = y_mea
        self.tau = thrust
        self.xi_est_l = self.xi_est.copy()
        self.v_est_l = self.v_est.copy()
        self.b_est_l = self.b_est.copy()
        self.y_low_l = self.y_low.copy()

        # Rotation matrix (使用 y_mea)
        psi_m = y_mea[2]
        self.J = np.array([
            [np.cos(psi_m), -np.sin(psi_m), 0],
            [np.sin(psi_m), np.cos(psi_m), 0],
            [0, 0, 1]
        ])

        if self.firsttime:
            self.y_est = y_mea.copy()
            self.y_low = y_mea.copy()
            self.y_low_l = y_mea.copy()
            self.firsttime = False
            self.JLow = self.J.copy()

    def WaveEstimator(self):
        dt = self.TP.dt
        self.xi_est = self.xi_est_l + dt * (self.Omega @ self.xi_est_l + self.K1 @ self.y_err)
        self.y_wave = self.varGamma @ self.xi_est

    def BiasEstimator(self):
        dt = self.TP.dt
        term1 = -np.linalg.solve(self.T, self.b_est_l)
        term2 = (1 / self.TP.gamma) * self.Lambda @ self.y_err
        self.b_est = self.b_est_l + dt * (term1 + term2)

    def JumpEstimator(self, RATIO):
        dt = self.TP.dt
        JumpCoeffMatrix = np.zeros((3, 3))
        ConstRatio = [0.5, 0.3, 0.3]

        JumpCoeffMatrix[0, 0] = (min(ConstRatio[0] * RATIO[0], self.TP.omegax) - self.TP.lambda_coeffx) * self.TP.k_til1
        JumpCoeffMatrix[1, 1] = (min(ConstRatio[1] * RATIO[1], self.TP.omegay) - self.TP.lambda_coeffy) * self.TP.k_til2
        JumpCoeffMatrix[2, 2] = (min(ConstRatio[2] * RATIO[2], self.TP.omegah) - self.TP.lambda_coeffh) * self.TP.k_til3

        self.b_jump = self.b_jump + dt * (1 / self.TP.gamma) * JumpCoeffMatrix @ self.y_err

    def VelocityEstimator(self):
        dt = self.TP.dt

        # 计算 RHS (括号内的项)
        RHS = (
                -self.VM.D @ self.v_est_l +
                self.JLow.T @ self.b_est +
                self.JLow.T @ self.b_jump +
                self.tau +
                (1 / self.TP.gamma) * self.JLow.T @ self.Ktil @ self.y_err
        )

        # 求解 A \ B
        # A = (obj.TP.dt * obj.VM.M)
        A = dt * self.VM.M

        # 求解 delta_v
        delta_v = np.linalg.solve(A, RHS)

        self.v_est = self.v_est_l + delta_v

    def LowEstimator(self):
        """ 参照 MATLAB NPOV4.m (文件 1) 中的 LowEstimator """
        dt = self.TP.dt
        self.y_low = self.y_low_l + dt * (self.JLow @ self.v_est + self.K2 @ self.y_err)