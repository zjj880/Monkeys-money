import socket
import threading
import time
import random


from tcp_worker import TCP_Worker
import para_def as para  # 导入参数定义模块

class TCP_Manager:

    def __init__(self):
        self.manager_port = para.MANAGERPORT
        self.ip_addr = para.IPADDR
        self.is_running = False
        self.worker_threads = []  # 用于跟踪活动的 Worker 线程
        self.port_lock = threading.Lock()
        self.reserved_ports = set()

        self.update_lock = threading.Lock()
        self.pending_lock = threading.Lock()  # 控制并发访问
        self.batch_cond = threading.Condition(self.pending_lock)
        self.update_event = threading.Event()
        self.update_required = False
        self.UPDATE_BATCH = 10  # 批次大小（10个worker的结果一起更新）
        self.UPDATE_EPOCH = 10
        self.current_batch = 0      # 当前进行到第几轮 / 第几批
        self.pkgs_in_batch = 0      # 当前这一轮已经收到多少个 pkg
        self.pending_pkgs = []  # 缓存worker上传的梯度包

    def get_available_port(self):
        """
        在 [SERVERPORT_MIN, SERVERPORT_MAX] 范围内寻找一个可用端口。
        使用互斥锁与“预定表”避免同进程内的并发重复分配。
        注意：这里仍然存在与外部进程的 TOCTOU 窗口，彻底规避需改 Worker 侧逻辑。
        """
        with self.port_lock:
            for port in range(para.SERVERPORT_MIN, para.SERVERPORT_MAX + 1):
                if port in self.reserved_ports:
                    continue
                try:
                    temp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                    # 在 Windows 上尽可能避免端口被同时拿到（可选，Linux 忽略）
                    try:
                        # 避免地址复用；并尽量独占（Windows 的 SO_EXCLUSIVEADDRUSE）
                        temp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
                        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                            temp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                    except Exception:
                        pass

                    temp_socket.bind((self.ip_addr, port))
                    # 这里仅用于“探测可用”，不进入 listen；立刻关闭
                    temp_socket.close()

                    # 逻辑上“预定”该端口，避免被本进程其他线程再次选中
                    self.reserved_ports.add(port)
                    return port

                except OSError:
                    # 端口不可用（被占用或无权限），尝试下一个
                    continue
                except Exception as e:
                    print(f"检查端口 {port} 时发生意外错误: {e}")
                    continue

        # 如果遍历完所有端口都没有找到可用的，则抛出异常
        raise Exception(f"在端口范围 {para.SERVERPORT_MIN} 到 {para.SERVERPORT_MAX} 内未找到可用端口。")

    def parse_data(self, data_str):
        # print(f"尝试解析数据: {data_str}")
        tokens = data_str.split(',')

        # 确保数据格式正确
        if not (tokens and tokens[0] == "$PARA"):
            raise ValueError("报文格式错误，非 $PARA 报文。")

        # 找到各个标记的位置
        try:
            thrust_number = int(tokens[1])
            target_pos_index = tokens.index("TARGET")
            freedom_index = tokens.index("FREEDOM")
            draught_index = tokens.index("DRAUGHT")
            gravity_index = tokens.index("GRAVITY")
            thrust_limit_index = tokens.index("THRUSTLIMIT")

            # 提取数据
            target_pos = [float(t) for t in tokens[target_pos_index + 1: target_pos_index + 4]]
            freedom = [int(t) for t in tokens[freedom_index + 1: freedom_index + 4]]
            draught = float(tokens[draught_index + 1])
            gravity = float(tokens[gravity_index + 1])
            thrust_limit = [float(t) for t in tokens[thrust_limit_index + 1: thrust_limit_index + 1 + thrust_number]]

            return {
                "ThrustNumber": thrust_number,
                "TargetPos": target_pos,
                "Freedom": freedom,
                "Draught": draught,
                "Gravity": gravity,
                "ThrustLimit": thrust_limit
            }

        except (ValueError, IndexError) as e:
            print(f"解析数据时发生错误: {e}")
            raise ValueError("数据内容缺失或类型错误。")

    def run_worker(self, worker_config):
        port = worker_config.get("WorkerPort")

        try:
            worker = TCP_Worker(worker_config, self.ip_addr)
            worker.loop()

        finally:
            with self.port_lock:
                self.reserved_ports.discard(port)


    def start_server(self):
        # 启动 Manager 服务器，长期运行并监听连接。
        self.is_running = True

        # 使用 socket.socket() 创建一个 TCP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            # 避免因为 TIME_WAIT 等导致复用问题（可选）
            try:
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except Exception:
                pass

            server_socket.bind((self.ip_addr, self.manager_port))
            server_socket.listen()  # 开始监听连接
            server_socket.settimeout(1.0)  # 设置超时，以便检查 is_running 标志
            print(f"TCP Manager 已启动，长期监听端口：{self.manager_port}")

            while self.is_running:
                try:
                    # 接受一个新的连接
                    conn, addr = server_socket.accept()
                    # print(f"接收到新的连接：{addr}")

                    with conn:
                        # 接收数据
                        data = conn.recv(1024)
                        if not data:
                            continue

                        data_str = data.decode('utf-8').strip()

                        if data_str.startswith("$PARA,"):
                            try:
                                # 1. 解析数据
                                client_params = self.parse_data(data_str)

                                # 2. 分配并“预定”端口（避免本进程内重复分配）
                                worker_port = self.get_available_port()

                                # 3. 构造 Worker 配置 (mngResults)
                                worker_config = {
                                    "WorkerPort": worker_port,
                                    **client_params
                                }

                                # 4. 发送新端口信息给客户端
                                response = f"$PORT,{worker_port},*"
                                conn.sendall(response.encode('utf-8'))
                                # print(f"已发送新端口：{response} 给 {addr}")

                                # 5. 在新线程中启动 Worker
                                worker_thread = threading.Thread(
                                    target=self.run_worker,
                                    args=(worker_config,),
                                    name=f"Worker-{worker_port}"
                                )
                                worker_thread.start()
                                self.worker_threads.append(worker_thread)

                                # 清理已完成的 Worker 线程
                                self.worker_threads = [t for t in self.worker_threads if t.is_alive()]

                            except ValueError as e:
                                print(f"处理客户端连接 {addr} 时发生错误: {e}")
                                error_response = "$ERROR,PARSE_FAIL,*"
                                conn.sendall(error_response.encode('utf-8'))
                            except Exception as e:
                                print(f"启动 Worker 线程时发生错误: {e}")
                                # 若启动失败，最好回滚预定（防止卡死），这里在 run_worker 的 finally 也会清理
                                error_response = "$ERROR,SERVER_FAIL,*"
                                conn.sendall(error_response.encode('utf-8'))
                        else:
                            print(f"收到未知报文：{data_str} from {addr}")

                except socket.timeout:
                    # 允许循环继续，检查 is_running
                    continue
                except KeyboardInterrupt:
                    self.is_running = False
                    print("服务器收到中断信号，正在关闭...")
                except Exception as e:
                    print(f"Manager 服务器发生意外错误: {e}")
                    time.sleep(1)  # 避免错误发生时 CPU 占用过高
        self.stop_server()

    def stop_server(self):
        # 关闭服务器并等待所有 Worker 线程结束。
        self.is_running = False
        for t in self.worker_threads:
            if t.is_alive():
                print(f"等待 {t.name} 线程结束...")
        print("所有 Worker 线程已清理。Manager 关闭完成。")
