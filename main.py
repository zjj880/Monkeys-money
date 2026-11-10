import sys
import os
import signal
from tcp_manager import TCP_Manager
import osqp

def signal_handler(sig, frame):
    """优雅地处理 Ctrl+C 中断"""
    print('\n收到 Ctrl+C，正在关闭服务器...')
    # Manager 实例将在 main_manager 变量中被访问，如果它存在
    if 'main_manager' in globals():
        globals()['main_manager'].stop_server()
    sys.exit(0)

if __name__ == "__main__":

    # 捕获 Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    # 实例化并运行 Manager
    print("----- 程序启动 -----")
    main_manager = TCP_Manager()

    try:
        main_manager.start_server()
    except Exception as e:
        print(f"主程序错误: {e}")
    finally:
        print("----- 程序退出 -----")