# KSKDP 与 OrcaFlex 11.4e 的 DLL 接口

DLL：`C:\Users\Administrator\Desktop\KSKDP\KSKDPExternalFunction.dll`

函数名：`CalcThrusterForce`（大小写保持一致）。仅用于 **64 位 OrcaFlex 的 Vessel applied loads**，不是 Python 脚本式外部函数，也不是船舶运动函数。

## 启动和配置

1. 双击本目录的 `run_kskdp.cmd`，看到 TCP Manager 监听端口 23011，保持窗口打开。
2. OrcaFlex 的 Variable data 中新建一个 **External function** 数据源，DLL 文件选择上面的绝对路径，函数名填写 `CalcThrusterForce`，初始值设为 0。
3. Vessel → Applied loads 中使用 **一行 Local applied loads**。将该行的 x、y、z force 与 x、y、z moment 六个分量都选择这个外部函数数据源。z force 将返回 0。五个非零分量依次是 Fx、Fy、Mx、My、Mz。
4. 这一行的作用点必须与 Python `para_def.py` 中推进器坐标采用的力矩参考点一致。如果这些坐标以船舶原点为参考，作用点填写 `(0,0,0)`。不要同时在其他 applied-load 行重复应用这组控制荷载。
5. 采用固定积分步长，使控制回调时刻包含每一秒，例如 0.1 s、0.05 s 或 1 s；DLL 每 1 s 计算一次并在期间保持指令。小步长仍应满足模型的积分精度要求。EKFV1.py 和 PID2.py 的 dt 均为 1 s；若要改变控制周期，须同步修改控制器与 DLL。
6. 配置下列船舶 Tags，然后复位模型，从头启动仿真。

## Vessel Tags

Tags 是船舶对象上的名称/值表，不是 Variable data 的函数名。默认连接参数已经与当前 `para_def.py` 一致；目标、吃水和重心高度须按实际模型设置。

| Tag 名称 | 默认值或示例 | 含义 |
|---|---|---|
| `KSKDP_IP` | `127.0.0.1` | 同一台电脑运行 Python 和 OrcaFlex。跨电脑时同步修改 Python IPADDR。 |
| `KSKDP_Port` | `23011` | TCP Manager 端口，与 Python MANAGERPORT 一致。 |
| `KSKDP_ThrusterCount` | `4` | 当前 Python 配置为 4 个推进器，不可仅修改此标签扩展推进器。 |
| `TargetX` | 按需求填写，例如 `0` | 目标全局 X，单位 m。不填时读取船舶 InitialX；若不能读取则为 0。 |
| `TargetY` | 按需求填写，例如 `0` | 目标全局 Y，单位 m。不填时读取船舶 InitialY；若不能读取则为 0。 |
| `TargetHeading` | 按需求填写，例如 `0` | 目标航向，单位 **度**。DLL 转换为 Python 所需弧度。不填时读取 InitialHeading。 |
| `Freedom` | `2,2,2` | X、Y、航向控制等级：0 关闭，1 为 0.5 倍，2 为 1 倍，3 为 1.5 倍。 |
| `Draught` | 默认 `0`，应填写实际吃水 | 吃水，m，传给 Worker；当前代码仅保存此参数。 |
| `GravityHeight` | 默认 `0`，应填写实际参考高度 | m。Python 用 `ThrusterZList - GravityHeight` 计算横摇、纵摇力臂；必须核对坐标基准。 |
| `ThrustLimit` | `1,1,1,1` | 各推进器能力比例，范围 0～1，数量必须为 4。 |
| `KSKDP_InitialAngles` | `0,0,1.5708,1.5708` | 初始反馈角，单位弧度，与 Python ThrusterAngle 一致。 |

DLL 可识别 local 和 global applied loads，global 返回值会用完整船舶姿态矩阵转换；推荐上面的 local 配置。不能同时叠加 local 与 global 两组相同推力。Python 的返回顺序是 `Fx,Fy,Mz,Mx,My`，DLL 已正确重排为 OrcaFlex 的六分量顺序。

Python 使用 m、kN、kN·m、rad。DLL 会将船舶位置转换为 m，将返回力和力矩转换为模型单位。姿态来自 Vessel 总运动（含波频运动），观测器再作滤波。

## 故障处理与运行限制

- 必须先启动 Python，再启动仿真。没有服务、连接关闭、报文不完整、数量不匹配或出现 NaN 时，DLL 通过 OrcaFlex 的错误报告 API 终止本次计算并给出原因，不读取空数据。
- Python 响应超时为 30 s，Worker 启动后的连接会短暂重试，处理 Manager 回复端口与 Worker 开始监听之间的时序。
- 同一船舶的各荷载分量共享一次控制计算，不会因六个分量而重复推进 EKF/PID。
- 目前不保存 Python 控制器状态，因此**不支持加载未完成的 .sim 后继续计算**；应复位并重新开始。仅仅暂停后继续、且 Python 和连接一直存在，不需要重建状态。
- 当前 Python Manager/Worker 接收端仍沿用原来的 `recv(1024)` 实现。DLL 接收端已经实现完整帧读取；跨机器或异常网络下，Python 接收端仍可能需要进一步加上分帧缓存。本次联调使用本机回环网络。

## 文件与验证

`dll_source` 目录保留 C 源文件、OrcaFlex 11.4 SDK 头文件和 `build_kskdp_dll.ps1`。DLL 仅依赖 Windows 系统库和系统 C 运行库，不需要给 OrcaFlex 配置 Python 引擎；Python 控制器单独运行。

使用 Zig 0.15.2 重编译示例：

```powershell
& '.\dll_source\build_kskdp_dll.ps1' -ZigPath 'C:\Users\Administrator\Documents\ChatGPT\新的尝试\zig-x86_64-windows-0.15.2\zig.exe'
```

已验证 64 位架构、三个导出入口、4/8 推进器报文、分段接收、一次计算多分量共享、local/global 转换、结果记录、断线、不完整报文、NaN、数量不匹配和 Manager 未启动。还通过真实 KSKDP Python 控制器进行了 10 次更新联调。

以上通信测试使用 11.4 SDK 布局的 API 测试替身，不等同于真实模型仿真。**尚未完成用户实际 OrcaFlex 模型的闭环仿真验证**，应在模型中按上面步骤验证荷载方向、参考点、目标位置与运动响应。

官方接口依据：[TExtFnInfo](https://www.orcina.com/webhelp/OrcFxAPI/Content/html/TExtFnInfo.htm)、[单位转换](https://www.orcina.com/webhelp/OrcFxAPI/Content/html/C_GetUnitsConversionFactor.htm)、[船舶姿态数据](https://www.orcina.com/webhelp/OrcFxAPI/Content/html/TVesselInstantaneousCalculationData.htm)。
