import numpy as np
from scipy.linalg import expm, ldl
import para_def as para  # 导入存放 MatrixM 和 MatrixD 的模块


class EKF:
    def __init__(self):
        # --- 调参结构体与噪声协方差 ---
        self.TP = {}
        self.Q = None
        self.R = None
        self.P = np.eye(15)

        # --- 系统状态与测量 ---
        self.y_mea = np.zeros((3, 1))
        self.y_est = np.zeros((3, 1))
        self.y_err = np.zeros((3, 1))
        self.x_est = np.zeros((15, 1))
        self.x_est_l = np.zeros((15, 1))
        self.x_pred = np.zeros((15, 1))
        self.tau = np.zeros((3, 1))

        # --- 时间与系统矩阵 ---
        self.dt = 1.0  # 对应 matlab 的 dt = 1
        self.A = None
        self.B = None
        self.E = None
        self.H = None
        self.M = None
        self.D = None
        self.firsttime = True

        # --- 初始化 ---
        self.SetDefaultParameters()
        self.init_system_matrices()

    def SetDefaultParameters(self):
        self.TP['q1'] = 0.001;
        self.TP['q2'] = 0.001;
        self.TP['q3'] = 0.0001
        self.TP['q4'] = 0.01;
        self.TP['q5'] = 0.01;
        self.TP['q6'] = 200
        self.TP['q7'] = 0.001;
        self.TP['q8'] = 0.001;
        self.TP['q9'] = 0.0001

        self.TP['r1'] = 0.002;
        self.TP['r2'] = 0.002;
        self.TP['r3'] = 0.0001
        self.UpdateQRMatrices()

    def UpdateQRMatrices(self):
        self.Q = np.diag([
            self.TP['q1'], self.TP['q2'], self.TP['q3'],
            self.TP['q4'], self.TP['q5'], self.TP['q6'],
            self.TP['q7'], self.TP['q8'], self.TP['q9']
        ])
        self.R = np.diag([self.TP['r1'], self.TP['r2'], self.TP['r3']])

    def init_system_matrices(self):
        w_01 = 2 * np.pi / 7
        xi_1 = 0.3
        A_21 = np.diag([-w_01 ** 2, -w_01 ** 2, -w_01 ** 2])
        A_22 = np.diag([-2 * xi_1 * w_01, -2 * xi_1 * w_01, -2 * xi_1 * w_01])

        self.A = np.zeros((15, 15))
        self.A[0:3, 3:6] = np.eye(3)
        self.A[3:6, 0:3] = A_21
        self.A[3:6, 3:6] = A_22

        T_b = np.diag([5000, 5000, 5000])
        self.A[9:12, 9:12] = -np.linalg.inv(T_b)

        # 对应 evalin('base', ...)
        self.M = para.MatrixM
        self.D = para.MatrixD
        # np.linalg.solve(M, D) 对应 M \ D
        self.A[12:15, 12:15] = -np.linalg.solve(self.M, self.D)

        self.B = np.zeros((15, 3))
        self.B[12:15, 0:3] = np.linalg.inv(self.M)

        self.H = np.zeros((3, 15))
        self.H[0:3, 3:6] = np.eye(3)
        self.H[0:3, 6:9] = np.eye(3)

        self.E = np.zeros((15, 9))
        self.E[3:6, 0:3] = np.eye(3)
        self.E[9:12, 3:6] = np.array([[6, 0, 0], [0, 9, 0], [0, 0, 200]])
        self.E[12:15, 6:9] = np.linalg.inv(self.M)

    def Rp_sai(self, p):
        p = float(p)
        return np.array([
            [np.cos(p), -np.sin(p), 0],
            [np.sin(p), np.cos(p), 0],
            [0, 0, 1]
        ])

    def EKFLoop(self, Pose, FDBForceLC):
        """对应 MATLAB 中的 EKF_DP_2Loop"""
        # 将传入参数统一重塑为 3x1 列向量，保证矩阵运算不出错
        y_mea = np.array(Pose).reshape(3, 1)
        control_input = np.array(FDBForceLC).reshape(3, 1)

        self.RefreshVariables(y_mea, control_input)
        self.predict()
        self.update()

    def RefreshVariables(self, y_mea, control_input):
        self.y_mea = y_mea
        self.tau = control_input
        self.x_est_l = self.x_est.copy()

        if self.firsttime:
            # 注意 Python 索引从 0 开始: 7:9 对应切片 6:9
            self.x_est[6:9, :] = y_mea
            self.x_est_l[6:9, :] = y_mea
            self.tau[0:3, :] = control_input
            self.firsttime = False

    def predict(self):
        p_sai = self.x_est[8, 0]  # x_est 的第 9 个元素
        R_mat = self.Rp_sai(p_sai)

        self.A[6:9, 12:15] = R_mat
        self.A[12:15, 9:12] = np.linalg.solve(self.M, R_mat.T)

        Phi = expm(self.A * self.dt)
        self.x_pred = Phi @ self.x_est_l + (self.B * self.dt) @ self.tau

    def update(self):
        epsilon = 1e-12
        Epsilon = epsilon * np.eye(self.P.shape[0])

        # 协方差预测
        Phi = expm(self.A * self.dt)
        E_dt = self.dt * self.E
        self.P = Phi @ self.P @ Phi.T + E_dt @ self.Q @ E_dt.T + Epsilon

        # 保证正定对称 (与 Matlab chol('lower') 对齐)
        try:
            S_pred = np.linalg.cholesky(self.P)
            self.P = S_pred @ S_pred.T
        except np.linalg.LinAlgError:
            # 防止极小概率的数值发散导致 Cholesky 报错
            self.P = (self.P + self.P.T) / 2

        P_pred = self.P.copy()

        # 卡尔曼增益 (使用LDL分解增强数值稳定性，原汁原味的 Matlab 逻辑)
        M2 = self.H @ self.P @ self.H.T + self.R
        L, D1, perm = ldl(M2)
        # 等效于 MATLAB 的 K = obj.P * obj.H' / (L * D1 * L')
        inv_M2 = np.linalg.inv(L @ D1 @ L.T)
        K = self.P @ self.H.T @ inv_M2

        # 状态更新
        y_pred = self.H @ self.x_pred
        y_error = self.y_mea - y_pred

        # 对应 MATLAB 的 wrapToPi(y_error(3))
        y_error[2, 0] = np.arctan2(np.sin(y_error[2, 0]), np.cos(y_error[2, 0]))

        self.x_est = self.x_pred + K @ y_error

        # 协方差更新 (Joseph form 形式)
        I = np.eye(15)
        temp = I - K @ self.H
        self.P = temp @ P_pred @ temp.T + K @ self.R @ K.T

    def GetLFMotion(self, step=None):
        """
        获取当前低频位移、速度与偏置 (大地系)
        外部调用: LFPose, LFVelocity, Bias = Observer.GetLFMotion(step)
        """
        # 提取状态量（此时变为一维数组 shape=(3,)）
        LFPosition = self.x_est[6:9, 0]
        LFVelocity_local = self.x_est[12:15, 0]
        b = self.x_est[9:12, 0]

        # 提取当前的艏向角 (psi)
        psi = self.x_est[8, 0]

        # 获取旋转矩阵 R(psi)
        R_mat = self.Rp_sai(psi)

        # 将局部系速度转换为大地系速度: V_global = R * V_local
        LFVelocity_global = R_mat @ LFVelocity_local

        return LFPosition, LFVelocity_global, b