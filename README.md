# KSKDP：Python 与 OrcaFlex 船舶定位闭环

本项目将 Python 的 EKF、PID 和推力分配与 OrcaFlex 船体模型连接，用于检查船舶在风浪流下的位置和航向保持。

## 当前基线

本次版本导入保留现有 Python 控制算法和参数。**尚未实施 EKF 专用艏摇惯量调整。** Transport 工况已完成一次原参数运行，出现持续艏摇振荡；诊断与试验计划见 `docs/transport-yaw-review.md`。

`models/SingleK_Transport_baseline.yml` 来自这次完成状态 SIM 的模型数据，仅调整了 DLL 相对路径和文件注释。原始 SIM、Excel、CSV 仍保留在本机原位置，未纳入普通 Git 提交。

## 代码入口

| 文件 | 作用 |
|---|---|
| `main.py` | 启动 TCP Manager |
| `tcp_manager.py` | 建立并管理控制会话 |
| `tcp_worker.py` | 执行观测、控制和推力分配 |
| `EKFV1.py` | 低频状态与偏置估计 |
| `pid_controller.py` | 位置、航向控制 |
| `para_def.py` | 质量、阻尼、控制增益和推进器参数 |
| `thrust_allocation2.py` | 推力分配 |
| `data_logger.py` | 记录 CSV 控制日志 |
| `dll_source/verified/KSKDPExternalFunction_verified.c` | 已完成闭环验证的 DLL 源码 |
| `dll_source/KSKDPExternalFunction.c` | 保留的原 DLL 源码，供历史对照 |

## 本机运行

目前已有环境可继续双击 `run_kskdp.cmd`。克隆到新目录后，使用 Python 3.12 创建 `.venv-kskdp`，安装 `requirements.txt`，再启动该脚本。端口配置见 `para_def.py`。

用 OrcaFlex 11.4 x64 打开模型。`models/SingleK_Transport_baseline.yml` 使用仓库根目录的 `KSKDPExternalFunction_verified.dll`。DLL 不纳入 Git；本机已留存运行副本，其他电脑需要重新编译或单独取得对应版本。

重新编译需要 OrcaFlex 11.4 的 `OrcFxAPI.h` 放在 `dll_source/sdk/`，以及 x64 MSVC 编译环境。在 x64 Native Tools Command Prompt 中运行 `dll_source\build_verified_dll.cmd`。SDK 和软件许可由安装环境提供。该新增编译入口尚未在本次导入中执行。

## 每次修改如何检查

1. 先提交已确认的基线，再建立一个 `codex/` 开头的试验分支。
2. 一次只改变一个明确因素，说明修改文件、位置、目的和验证结果。
3. 查看 `git diff`：红色是删除或替换前的行，绿色是新增或替换后的行。
4. 验证后单独提交，用提交号对应本次模型、参数和结果。
5. 将代码检查、简化模型诊断和完整 OrcaFlex 仿真分开记录。

常用只读命令：

```text
git status
git diff
git diff --cached
git log --oneline --decorate
git show <提交号>
```

GitHub Desktop 的 Changes 显示未提交改动，History 显示已提交的版本和逐文件差异。每次修改后，反馈应包含：提交号、文件和行号、实际改动、检查结果、仍未验证的事项。

## 模型与结果记录

- 文本模型、参数、源码、简要统计和说明纳入版本管理。
- `.sim`、原始 CSV、Excel 和虚拟环境留在本地；需要另行归档，它们不会随普通 Git push 自动备份。
- 已有原始日志及 SIM 的校验值记录在 `docs/baseline-manifest.json`，便于确认工况和代码对应关系。
- 每次仿真记录代码提交号、模型文件、随机种子、运行区间、日志文件和主要误差指标。

## 当前下一步

先建立并提交当前基线，再单独创建“EKF 艏摇惯量对齐”试验。观测器矩阵需要与 PID 增益生成来源分开，避免改动 `MatrixM` 时同时改变 Kp、Kd。试验前后同时比较 X、Y 和 yaw，不能只看航向曲线。
