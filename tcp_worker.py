import socket
import time

import para_def as para
import numpy as np
from npo_v4 import NPOV4
from EKFV1 import EKF
from pid_controller import PID2
from thrust_allocation2 import QP2
from data_logger import DataLogger
from refFilter import REFFilter


# ==============================================================================
# === TCP_Worker 类实现 ===
# ==============================================================================

class TCP_Worker:

    def __init__(self, worker_config, ip_addr):
        # 初始化 Worker
        self.worker_port = worker_config["WorkerPort"]
        self.ip_addr = ip_addr
        self.config = worker_config

        # 从 para_def 或 worker_config 获取推进器数量
        self.num_thrusters = len(para.ThrusterXList)

        # 初始化所有属性
        self.FinalTargetPose = worker_config["TargetPos"]
        self.TargetPose = np.zeros(3)
        self.draught = worker_config["Draught"]
        self.gravityHeight = worker_config["Gravity"]
        self.thrustUsage = worker_config["ThrustLimit"]
        self.controlLevel = worker_config["Freedom"]
        self.TargetVel = np.zeros(3)
        self.Pose = np.zeros(3)
        self.LFPose = np.zeros(3)
        self.LFVelocity = np.zeros(3)
        self.Bias = np.zeros(3)
        self.FDBThrust = np.zeros(self.num_thrusters)
        self.FDBAngle = np.zeros(self.num_thrusters)
        self.FDBForceLC = np.zeros(3)
        self.CMDForce = np.zeros(3)
        self.CMDForceLC = np.zeros(3)
        self.CMDForceLC5D = np.zeros(5)
        self.CMDThrust = np.zeros(self.num_thrusters)
        self.CMDAngle = np.zeros(self.num_thrusters)
        self.step = 0  # MATLAB 中初始化为 0
        self.PostPro = 0  # 0-不记录

        # 初始化组件
        self.ReferenceFilter = REFFilter(self.FinalTargetPose)
        #self.Observer = NPOV4()
        #self.Observer.SetPara(para.MatrixM * 1000, para.MatrixD * 1000)
        self.Observer = EKF()
        self.Controller = PID2()
        self.Controller.SetPara(self.controlLevel)
        self.ThrustAllocation = QP2()
        self.ThrustAllocation.SetPara(self.gravityHeight, self.thrustUsage)
        self.logger = DataLogger(self.worker_port, self.num_thrusters)

        # print(f"Worker 实例创建，端口: {self.worker_port}, 配置: {self.config}")

    def loop(self):
        timeout = para.WORKERTIMEOUT
        try:
            # 1. 创建并绑定监听套接字
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind((self.ip_addr, self.worker_port))
                s.listen(1)  # 监听一个连接
                # ** 设置 s.accept() 的超时，防止 Worker 启动后客户端不连接 **
                s.settimeout(timeout)

                # 2. 接受客户端连接
                conn, addr = s.accept()
                # print(f"[Worker {self.worker_port}] 客户端 {addr} 已连接。开始循环。")

                # 3. 开始主循环 (接收数据、计算、发送响应)
                with conn:
                    # ** 设置 conn.recv() 的超时， 防止 Worker 联通后数据不进来 **
                    conn.settimeout(timeout)

                    while True:
                        # 接收数据 (阻塞，直到收到数据或超时)
                        data = conn.recv(1024)
                        if not data:
                            # 客户端主动断开连接
                            # print(f"[Worker {self.worker_port}] 客户端断开连接。退出循环。")
                            break
                        self.step += 1
                        received_message = data.decode('utf-8').strip()

                        try:
                            self.recvPose(received_message)
                            self.calcForce()
                            self.sendForce(conn)

                        except Exception as parse_e:
                            print(f"[Worker {self.worker_port}] 处理数据时发生错误: {parse_e}")


        except socket.timeout:
            # 捕获 s.accept() 超时 (无连接) 或 conn.recv() 超时 (无数据)
            # print(f"[Worker {self.worker_port}] 连接/数据接收超时 ({timeout}s)，释放端口。")
            pass
        except Exception as e:
            print(f"[Worker {self.worker_port}] 发生错误: {e}")
        finally:
            return

    def recvPose(self, data_str):
        tokens = data_str.split(',')

        try:
            poseIdx = tokens.index("$POSE") + 1                                                 # 解析 Pose
            self.Pose = np.array([float(t) for t in tokens[poseIdx:poseIdx + 3]])

            thrustIdx = tokens.index("$THRUST") + 1                                             # 解析 Thrust
            self.FDBThrust = np.array([float(t) for t in tokens[thrustIdx: thrustIdx + self.num_thrusters]])

            angleIdx = tokens.index("$ANGLE") + 1                                               # 解析 Angle
            self.FDBAngle = np.array([float(t) for t in tokens[angleIdx: angleIdx + self.num_thrusters]])

        except (ValueError, IndexError) as e:
            print(f"[Worker {self.worker_port}] 解析报文失败: {e}. 报文: {data_str}")
            raise ValueError("报文解析错误")

    def calcForce(self):
        # 1. 状态观测器
        self.FDBForceLC = self.CMDForceLC5D[:3]                                                   # 上一帧的 CMDForceLC。
        #self.Observer.NPOLoop(self.Pose, self.FDBForceLC * 1000)
        #self.LFPose, self.LFVelocity, self.Bias = self.Observer.GetLFMotion(self.step)
        self.Observer.EKFLoop(self.Pose, self.FDBForceLC)
        self.LFPose, self.LFVelocity, self.Bias = self.Observer.GetLFMotion(self.step)
        # 2. 目标滤波器
        # self.TargetPose, self.TargetVel = self.ReferenceFilter.Loop(self.Pose, self.LFVelocity)   # 有参考滤波器
        self.TargetPose = self.FinalTargetPose
        self.TargetVel = np.zeros(3)
        # 3. 控制器
        self.CMDForce, self.CMDForceLC = self.Controller.Loop(self.TargetPose, self.TargetVel, self.LFPose, self.LFVelocity, self.Bias)
        # 4. 推力分配
        success = self.ThrustAllocation.QPLoop(self.CMDForceLC, self.Pose)
        if success:
            self.CMDForceLC5D, self.CMDThrust, self.CMDAngle = self.ThrustAllocation.getState()
        else:
            self.CMDForceLC5D, self.CMDThrust, self.CMDAngle = self.ThrustAllocation.getState()

    def sendForce(self, conn):
        # 格式化并发送计算结果给客户端。
        thrustValues = ",".join([f"{v:.4f}" for v in self.CMDThrust])  # 生成 THRUST 部分的数据
        angleValues = ",".join([f"{v:.4f}" for v in self.CMDAngle])  # 生成 ANGLE 部分的数据

        # 组合最终报文 并发送
        response = (
            f"$FORCE,{self.CMDForceLC5D[0]:.2f},{self.CMDForceLC5D[1]:.2f},{self.CMDForceLC5D[2]:.2f},{self.CMDForceLC5D[3]:.2f},{self.CMDForceLC5D[4]:.2f},"
            f"THRUST,{thrustValues},"
            f"ANGLE,{angleValues},*"
        )
        conn.sendall(response.encode('utf-8'))

        self.logger.log_step(
            self.step,
            self.FinalTargetPose,
            self.TargetPose,
            self.TargetVel,
            self.Pose,
            self.LFPose,
            self.LFVelocity,
            self.Bias,
            self.CMDForceLC5D[:3],
            self.CMDForce[:3],
            self.CMDThrust
        )

