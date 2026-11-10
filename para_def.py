import numpy as np
import math

# %% Server IP (保持原有格式)
IPADDR = "127.0.0.1"
MANAGERPORT = 23011
SERVERPORT_MIN = 23012
SERVERPORT_MAX = 23021
WORKERTIMEOUT = 50

# %% Vessel (t, t·m^2) (使用新数据，保持 NumPy 数组)
MatrixM = np.array([
    [32.58 * 1e3 * 1.2, 0, 0],
    [0, 32.58 * 1e3 * 2.0, -10 * 1e3],
    [0, -10 * 1e3, 56 * 1e6 * 2]
], dtype=np.float64)

MatrixD = np.array([
    [300, 0, 0],
    [0, 600, 0],
    [0, 0, 15 * 10**5]
], dtype=np.float64)

# %% Controller (使用新数据和新逻辑)
Kp_vec = np.zeros(3, dtype=np.float64)
Kd_vec = np.zeros(3, dtype=np.float64)

# 循环计算 Kp 和 Kd
for i in range(3):
    Kp_vec[i] = MatrixM[i, i] * 6.5e-3
    Kd_vec[i] = MatrixM[i, i] * 1.5e-1
Kp_vec[1] = 2 * Kp_vec[1]
Kp_vec[2] = 2 * Kp_vec[2]


# 直接使用 NumPy 数组
Kp = Kp_vec  # 形状 (3,), 类型 float64
Kd = Kd_vec  # 形状 (3,), 类型 float64

# Ki = [0, 0, 0] 转换为 NumPy 数组
Ki = np.array([0, 0, 0], dtype=np.float64)  # 形状 (3,)

# FLim = [700, 700, 30000] 转换为 NumPy 数组
FLim = np.array([450, 900, 20000], dtype=np.float64)  # 形状 (3,)

# %% Thruster (使用新数据，转换为 NumPy 数组)
ThrusterXList = np.array([-62.3, -62.3, 76.95, 76.95], dtype=np.float64)  # 形状 (6,)
ThrusterYList = np.array([8.0, -8.0, 0.0, 0.0], dtype=np.float64)  # 形状 (6,)
ThrusterZList = np.array([-7.0, -7.0, -7.0, -7.0], dtype=np.float64)
ThrusterAngle = np.array([0.0, 0.0, 1.5708, 1.5708], dtype=np.float64)  # 形状 (6,)

# 100% Max Force Newest (列向量版本)
ThrusterMaxTList = np.array([500.0, 500.0, 235.0, 235.0], dtype=np.float64)*2  # 形状 (6,)
ThrusterMinTList = np.array([-500.0, -500.0, -235.0, -235.0], dtype=np.float64)  # 形状 (6,)
ThrusterMaxDTList = np.array([300.0, 300.0, 200.0, 200.0], dtype=np.float64)  # 形状 (6,)
ThrusterMaxAlphaList = np.array([3.14159, 3.14159, 1.5709, 1.5709], dtype=np.float64)  # 形状 (6,)
ThrusterMinAlphaList = np.array([-3.14159, -3.14159, 1.5707, 1.5707], dtype=np.float64)  # 形状 (6,)
ThrusterMaxDAlphaList = np.array([3.14/180*10.0, 3.14/180*10.0, 0.001, 0.001], dtype=np.float64)  # 形状 (6,)

# %% Weight-QP2 (保持标量)
W_W2 = 10             # Weight para for energy saving
W_Omega2 = 10         # Weight para for rate of angle change
W_Q2 = 10000000           # Weight para for error between target and solved

# %% Reference Filter (使用新数据，转换为 NumPy 数组)
RefOmega = 2 * math.pi / 200 * np.eye(3, dtype=np.float64)  # 形状 (3, 3)
RefDelta = 0.9 * np.eye(3, dtype=np.float64)                # 形状 (3, 3)
RefMaxV = np.array([0.2, 0.15, 0.1 / 180 * math.pi], dtype=np.float64)  # 形状 (3,)
# %% QP anti-chattering / nonlinear-safeguard parameters
# 说明：以下参数由 thrust_allocation2_fixed.py 读取；若不加入 para_def.py，代码也会使用内置默认值。
# W_DeltaT2 越大，推进器推力变化越平滑，但响应会更慢；建议 300~2000 间调试。
W_DeltaT2 = 20.0

# W_DeltaAlpha2 越大，方位角变化越平滑；建议不小于 W_Omega2。
W_DeltaAlpha2 = W_Omega2

# 松弛变量限幅：surge, sway, yaw。通常 yaw 量纲更大。
QP_SlackBound = np.array([1.0e3, 1.0e3, 1.0e5], dtype=np.float64)

# 开启非线性回退：当线性化QP给出的全步长导致真实合力误差变大时，自动缩短步长。
QP_Backtracking = True
QP_BacktrackingScales = [1.0, 0.5, 0.25, 0.125]

# OSQP 参数
QP_MaxIter = 16000
QP_Polish = True
QP_EpsAbs = 1e-3
QP_EpsRel = 1e-3

# %% Extra robust QP anti-freeze parameters
# 允许回退候选步长的真实误差比上一拍略大一点点，避免因 0.999 严格门槛导致卡死。
QP_BacktrackingAcceptTol = 5.0e-3

# 当所有候选步长都没有达到接受条件时，仍施加最小步长，避免输出长期维持上一周期。
QP_ForceSmallStepWhenStalled = True

# 调试时打开，会打印 forced_small_step / fallback 信息。
QP_Debug = False
