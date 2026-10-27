# Teleop Studio 用户使用文档

> 遥操作控制台 — Moretion 动捕 / GMR 重定向 / SONIC 全身控制

Teleop Studio 是一个基于 PySide6 的桌面图形界面程序，用于实时重定向人体动作捕捉数据，并通过 SONIC 控制 MuJoCo 仿真器或 Unitree G1 人形机器人。它把「Moretion 动捕串流 → GMR 运动重定向 → SONIC 全身控制」整条流水线封装为一个统一的控制面板，操作员只需在界面上完成参数填写、启动重定向、连接 SONIC、开始/暂停跟踪即可。

---

## 目录

1. [系统组成与原理](#1-系统组成与原理)
2. [运行环境与硬件准备](#2-运行环境与硬件准备)
3. [安装与启动](#3-安装与启动)
   - [MuJoCo 仿真启动](#mujoco-仿真启动)
4. [界面总览](#4-界面总览)
5. [参数配置详解](#5-参数配置详解)
6. [状态指示灯](#6-状态指示灯)
7. [控制按钮](#7-控制按钮)
8. [日志区](#8-日志区)
9. [标准操作流程](#9-标准操作流程)
10. [安全与急停](#10-安全与急停)
11. [常见问题（FAQ）](#11-常见问题faq)

---

## 1. 系统组成与原理

Teleop Studio 把三个子系统串成一条实时流水线：

```
┌──────────────┐    ┌──────────────────┐    ZMQ(50Hz)   ┌────────────────────┐
│ Moretion 动捕 │ → │ GMR 重定向(本机)  │ ───────────→  │ SONIC 全身控制      │
│  服务器        │    │ moretion_stream_  │   pose 主题   │ （本机或远程主机）   │
└──────────────┘    │  to_sonic.py      │               │  deploy.sh         │
 TCP/UDP 9999       └──────────────────┘               └────────────────────┘
                                                               │
                                                               ▼
                                                     MuJoCo / Unitree G1
```

- **Moretion 动捕服务器**：每帧输出人体骨骼位姿（关节欧拉角 + 根位置），通过 TCP 或 UDP 串流到本机。
- **GMR 重定向（本机子进程）**：接收动捕帧，做 IK 重定向得到机器人关节角，以 50 Hz 通过 ZMQ PUB 发布 `pose` 主题。
- **SONIC 全身控制（部署主机）**：在本机或远程 Linux 主机上运行 `deploy.sh`，订阅 ZMQ 的 `pose`，控制 MuJoCo 仿真器或向真机下发电机指令。

Teleop Studio 界面本身只负责「参数填写 + 启动/停止 + 状态监控」，不直接参与数据传输。

**关键点**：远程 SONIC 部署主机需要能通过网络访问操作机的 ZMQ 端口（默认 5556），因此「本机网络接口」必须选择远程主机可访问的网段。本地部署固定使用 `localhost`。

---

## 2. 运行环境与硬件准备

### 2.1 操作系统

- Linux / Ubuntu（推荐，本地部署模式仅 Linux 可用）
- Windows（仅支持远程 SSH 部署模式）

### 2.2 Python 环境

需要 conda 环境 `gmr`（开发态），或随附的 conda-pack 打包环境（打包态）。启动脚本会自动探测并激活，无需手动处理。

### 2.3 硬件清单

| 设备 | 说明 |
|------|------|
| 人形机器人 | Unitree G1（29 自由度），默认 SSH 地址 `192.168.3.9`，用户名 `unitree` |
| 动捕服务器 | Moretion，默认 IP `192.168.2.132`，端口 `9999` |
| 操作机（PC） | 运行 Teleop Studio 的电脑，需能访问动捕服务器，并按部署方式访问远程主机或机器人网段 |

### 2.4 网络

- 操作机必须能连通动捕服务器；远程部署时还需连通远程 Linux 主机。
- 程序默认优先选中 `192.168.3.*`（常用机器人网段）的网卡作为 ZMQ Host，让远程部署主机能连接回操作机。
- 若使用远程 SSH 模式，操作机需能 SSH 到所填写的 SSH 主机。

---

## 3. 安装与启动

### 3.1 Linux / Ubuntu

```bash
bash scripts/run_teleop_studio.sh
```

启动脚本会自动判断运行模式：

- **打包态**：脚本同级存在 `env/`（conda-pack 环境）与 `gmr/`（仓库），首次运行自动执行 `conda-unpack`，之后激活并启动。
- **开发态**：无 `env/` 同级目录，回退到激活 conda 的 `gmr` 环境，仓库取上级目录。

脚本会自动把 PySide6 自带的 Qt6 库目录前置到 `LD_LIBRARY_PATH`，避免系统 Qt6 被优先加载导致符号缺失报错。

### 3.2 Windows

```powershell
./scripts/run_teleop_studio.ps1
```

同样支持打包态与开发态。开发态会自动探测 conda（PATH / 注册表 / 常见安装路径），优先使用 `gmr` 环境，回退到 `base`。

可通过环境变量覆盖：

- `CONDA_ROOT`：conda 安装目录
- `CONDA_ENV`（Windows）：conda 环境名，默认 `gmr`

启动后弹出窗口，中文标题为 `Teleop Studio - moretion / GMR / SONIC 遥操作`，初始尺寸为 1100×760。

---

### MuJoCo 仿真启动

SONIC 仿真器可以独立于 Teleop Studio 启动，用于在 Ubuntu 主机上验证 MuJoCo、DDS 和 G1 模型。启动脚本会自动定位当前 `gmr` Conda 环境；如果脚本所在目录向上能找到 `env/` 和 `gmr/`，则使用 `conda-pack` 解压后的环境，并在首次运行时执行 `conda-unpack` 修复移动后的路径。

在 GMR 仓库根目录执行：

```bash
chmod +x scripts/run_sonic_sim.sh
bash scripts/run_sonic_sim.sh
```

无显示器或 SSH 会话使用：

```bash
bash scripts/run_sonic_sim.sh --no-onscreen
```

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--interface sim` | `sim` | 使用回环接口 `lo`，本机 SONIC/WBC 联调时使用 |
| `--interface <网卡名或 IP>` | | 跨主机 DDS 联调时指定可达网卡 |
| `--sim-frequency 200` | `200` | MuJoCo 仿真频率（Hz） |
| `--control-frequency 50` | `50` | 控制频率（Hz） |
| `--no-onscreen` | 开启窗口 | 无显示器时禁用 MuJoCo viewer |
| `--offscreen --enable-image-publish` | 关闭 | 开启相机渲染和 ZMQ 图像发布 |

仿真器启动后，在另一个终端启动 `gear_sonic_deploy` 的 WBC。两端需要使用相同的 CycloneDDS 版本、Domain ID 和网络接口；本机联调通常保持 `--interface sim`：

```bash
# 终端 1：GMR 仿真
bash scripts/run_sonic_sim.sh --interface sim

# 终端 2：SONIC WBC（示例）
cd /path/to/gear_sonic_deploy
bash deploy.sh --input-type zmq \
  --zmq-host 127.0.0.1 --zmq-port 5556 --zmq-topic pose sim
```

`run_sonic_sim.sh` 会优先设置 `sonic_sim/lib/libddsc.so`，并把 GMR 与 `third_party/unitree_sdk2_python` 加入 `PYTHONPATH`。不要直接使用系统 Python 启动 `scripts/run_sonic_sim.py`，否则可能加载错误的 DDS 库或找不到 Unitree SDK。

移植到其他 Ubuntu 主机时，只需复制包含 `env/`、`gmr/` 和本启动脚本的目录。目标机应与打包机使用相同 CPU 架构（通常为 `x86_64`）和兼容的 Ubuntu/glibc；无显示器时使用 `--no-onscreen`。启用 offscreen 渲染仍需目标机提供 MuJoCo 所需的 OpenGL/EGL 运行库。DDS 的 `DOMAIN_ID`、`--interface` 和防火墙设置必须与 WBC 端一致；没有 WBC 发布 `rt/lowcmd` 时，模型保持默认/等待状态属于正常现象。

## 4. 界面总览

界面自上而下分为 4 个分组：

1. **运行参数**：GMR 与网络、部署相关配置
2. **状态指示**：4 个彩色圆点状态灯
3. **控制按钮**：重定向、机器人、跟踪 3 个按钮
4. **日志区**：GMR 输出 与 SONIC 输出 两个只读文本框

菜单栏中的「语言」菜单可在「中文」和「English」之间即时切换，不会清空已填写的参数。所选语言会被保存，并在下次启动时继续使用。

---

## 5. 参数配置详解

### 5.1 重定向（GMR）参数

| 控件 | 说明 | 默认值 |
|------|------|--------|
| GMR Python | 运行重定向脚本所用的 Python 可执行文件绝对路径（conda 环境） | 当前 `sys.executable` |
| GMR 仓库目录 | GMR 仓库根目录，用于定位 `scripts/moretion_stream_to_sonic.py` | 根据 `teleop_studio/` 所在位置自动确定 |
| Moretion 地址 | 动捕服务器 IP | `192.168.2.132` |
| Moretion 端口 | 动捕串流端口（1–65535） | `9999` |
| UDP | 勾选则用 UDP 替代 TCP 接收动捕数据 | 关闭 |
| 保留重定向窗口 | 勾选则保留 MuJoCo 可视化窗口（便于观察重定向效果） | 开启 |

### 5.2 网络接口

| 控件 | 说明 |
|------|------|
| 本机网络接口 | 下拉枚举本机所有 IPv4 接口（含 WSL），过滤掉 loopback 与 link-local。右侧「刷新」按钮可重新枚举 |
| SONIC ZMQ Host | 由所选网络接口的 IP 自动填充；本地部署模式下固定为 `localhost` |
| SONIC ZMQ 端口 | ZMQ 发布端口，默认 `5556` |

> 提示：默认优先选中 `192.168.3.*` 网段（常用机器人网段）。若未自动选中，请手动选择远程 SONIC 部署主机可访问的网卡。

### 5.3 部署与控制

下拉框两个选项：

- **远程 SSH（在远程主机部署 SONIC）**：通过 SSH 连接 Linux 主机并在其上执行 `deploy.sh`。Windows/Linux 均可用。
- **本地部署（SONIC 在本机，有线连机器人）**：在本机用 PTY 跑 `deploy.sh`，仅 Linux 可用。

部署位置与运行目标相互独立。SONIC 控制仿真器时选择默认的 **MuJoCo 仿真**，包括从 Windows 通过 SSH 启动 Linux 仿真的场景；只有部署到真实硬件时才选择 **Unitree G1 真机**。本地和远程部署均可选择这两个目标。

| 控件 | 选项与作用 | 默认值 |
|------|------------|--------|
| 运行目标 | **MuJoCo 仿真**会向 `deploy.sh` 传入 `sim`；**Unitree G1 真机**会传入 `real` | MuJoCo 仿真 |
| 控制模型 | **默认模型**使用 SONIC 的默认策略；**低延迟模型**使用 `policy/low_latency/model` 与对应的 `observation_config.yaml` | 默认模型 |
| 动作预览延迟 | 已收到的 GMR 动作用作真实预览，未知未来尾部保持姿态；`-1` 为完整预览（920 / 200 ms） | 默认模型 400 ms；低延迟模型 100 ms |

切换模型会重设为对应折中延迟。只运行重定向时可直接修改这两项，参数实时同步，
无需重启重定向窗口；部署启动后锁定，确保发送端与模型一致。每包保留 1 帧历史、
目标帧和模型所需的未来参考；`--batch 2` 只统计历史帧与目标帧。详见 `SONIC_LATENCY.md`。

### 5.4 SSH / 远程部署（仅远程 SSH 模式启用）

| 控件 | 默认值 |
|------|--------|
| SSH 主机 | `192.168.3.9` |
| SSH 用户 | `unitree` |
| SSH 密码 | （留空，密码模式） |
| SSH 私钥 | 可选，留空则用密码登录 |
| 部署目录 | 远程：`/home/unitree/workspace/GR00T-WholeBodyControl/gear_sonic_deploy`；本地：`/home/zpf/workspace/GR00T-WholeBodyControl/gear_sonic_deploy` |

---

## 6. 状态指示灯

界面中部有 4 个彩色圆点状态灯：

| 状态灯 | 颜色含义 |
|--------|----------|
| GMR 重定向 | 灰=空闲，蓝=运行中 |
| SSH 连接 / 本地部署 | 灰=空闲，绿=已连接 |
| 控制状态 | 橙=等待控制，绿=控制中，灰=未知 |
| 跟踪状态 | 绿=跟踪中，橙=已暂停，灰=未知 |

其中「控制状态」「跟踪状态」由 SONIC 部署进程的日志关键字自动解析回写，无需手动操作。

---

## 7. 控制按钮

三个按钮，颜色会随状态变化。过渡态会变灰并禁用，避免误触。

| 按钮 | 各状态显示与含义 |
|------|------------------|
| 重定向 | 绿色「开始」→（启动后）红色「停止」 |
| 机器人 | 绿色「连接」→（控制中）红色「急停」 |
| 跟踪 | 绿色「开始\n跟踪」→（跟踪中）红色「暂停\n跟踪」。**仅在机器人已连接且控制中时可用** |

跟踪按钮是倒角正方形大按钮，跨两行，便于操作员快速点击。

---

## 8. 日志区

两个只读文本框，最多保留 5000 行，等宽字体：

- **GMR 输出**：重定向子进程的 stdout / stderr，实时滚动。
- **SONIC 输出**：远程 SSH shell 或本地 PTY 的输出，实时滚动。

日志是排查问题的主要依据，出现异常时优先查看这两个区域。

---

## 9. 标准操作流程

一次完整的遥操作任务按以下步骤进行。

### 步骤 0：准备 Moretion 动捕服务器

- 在 Moretion 服务器上启动动捕串流。
- 可使用实时动作捕捉或预录动作回放作为输入。

### 步骤 1：启动应用

```bash
bash scripts/run_teleop_studio.sh      # Linux
./scripts/run_teleop_studio.ps1        # Windows PowerShell
```

### 步骤 2：确认 / 填写运行参数

- 检查 GMR Python 与 GMR 仓库目录是否正确。
- 填入 Moretion 动捕服务器 IP 与端口（确认动捕服务器已开始串流）。
- 选择正确的本机网络接口（远程 SONIC 部署主机必须能访问其 IP）。
- 选择部署模式：
  - 远程 SSH：填入 SSH 主机 / 用户 / 密码或私钥、部署目录。
  - 本地部署：确认部署目录正确。
- 选择运行目标：Sonic Sim 保持 **MuJoCo 仿真**；仅真机部署选择 **Unitree G1 真机**。
- 选择控制模型：通常保持 **默认模型**；需要使用低延迟策略时选择 **低延迟模型**。

### 步骤 3：启动 GMR 重定向

1. 确保动捕服务器已就绪并开始串流。
2. 点击绿色「开始」重定向按钮。
3. 程序会以子进程启动 `moretion_stream_to_sonic.py`：
   - 连接 Moretion 动捕服务器，等待第一帧并自动检测人体身高。
   - 初始化 IK 重定向器与（可选）MuJoCo 可视化窗口。
   - 以 50 Hz 通过 ZMQ 发布 `pose` 主题到 `tcp://*:5556`。
4. 重定向状态灯变蓝「运行中」，按钮变红「停止」，GMR 输出日志开始滚动。

### 步骤 3.1：完成实机前验证

连接真实机器人前必须依次完成以下检查，任一阶段出现异常时不得进入下一阶段：

1. 保持「保留重定向窗口」开启，观察重定向参考动作是否接近人体动作，重点检查左右侧、关节转向、动作幅度以及姿态是否连续，确认没有突跳、反向或明显失真。
2. 将运行目标设为 **MuJoCo 仿真**，使用计划在实机上执行的动作进行完整测试，确认机器人姿态、平衡和关节运动正常，并验证暂停跟踪与急停功能有效。
3. 仿真确认正常后，方可将运行目标切换为 **Unitree G1 真机**。首次实机测试应使用小幅、缓慢动作并缩短测试时间，确认稳定后再逐步增加动作幅度和持续时间。

### 步骤 4：连接 SONIC / 机器人

1. 点击绿色「连接」机器人按钮。
   - 远程 SSH 模式：通过 paramiko 建立 SSH 交互式 shell，并发送部署命令：
     ```
     cd <部署目录> && bash deploy.sh [低延迟模型参数] --input-type zmq \
       --zmq-host <本机WiFi IP> --zmq-port <sonic_port> \
       --zmq-topic pose <sim|real>
     ```
     选择低延迟模型时，`[低延迟模型参数]` 为 `--cp policy/low_latency/model --obs-config policy/low_latency/observation_config.yaml`；默认模型不添加这两个参数。
   - 本地部署模式：在本机用 PTY 执行同样的部署命令。
  
2. 程序会自动处理 deploy.sh 的交互提示（自动回答 yes 并发送 `]` 进入控制系统）。
3. 当日志出现 `transitioning to control state` 时，控制状态灯变绿「控制中」，机器人按钮变红「急停」。

### 步骤 5：开始跟踪

1. 确认机器人已进入控制中状态（控制状态灯为绿）。
2. 此时跟踪按钮可用，点击绿色「开始\n跟踪」大按钮。
3. 程序会向 deploy shell 发送回车键，切换 SONIC 的 planner。当日志出现 `planner initialized successfully` 时，跟踪状态灯变绿「跟踪中」，按钮变红「暂停\n跟踪」。
4. 此时动捕数据已实时发送到所选运行目标，开始遥操作或仿真。

### 步骤 6：暂停 / 恢复跟踪

- 跟踪中点击红色「暂停\n跟踪」可暂停 planner（日志出现 `disabling planner`，跟踪状态灯变橙「已暂停」）。
- 再次点击绿色按钮即可恢复。

### 步骤 7：结束任务

1. 先点击「暂停\n跟踪」暂停跟踪。
2. 点击红色「急停」停止 SONIC 控制（发送 `O` 键触发急停）。
3. 点击红色「停止」停止 GMR 重定向子进程：
   - Linux 发送 SIGTERM，3 秒超时保底 kill。
   - Windows 发送 CTRL_BREAK_EVENT。
   - 子进程退出前会发送 `stop` 命令、关闭 receiver / viewer。

---

## 10. 安全与急停

> **警告：** 真机遥操作存在机器人跌倒、失控或意外动作的风险。软件限位和急停不能替代现场安全防护；未完成本节检查时，不得启动真机跟踪。

### 10.1 吊装与场地防护

- 平时调试和遥操作时，应使用额定载荷满足要求的吊架或安全绳悬挂机器人，限制机器人跌倒范围。每次使用前检查吊架、吊点、绳索和连接件是否牢固，并确保吊装不会限制正常动作或拉扯线缆。
- 禁止人员站在悬吊机器人下方。机器人周围应划定安全区域，清除桌椅、线缆和其他障碍物；操作员及其他人员始终与机器人保持安全距离，不得进入机器人可能倾倒或肢体可能扫过的范围。
- 真机测试时至少安排一名人员持续观察机器人和周围环境。确保急停操作端和机器人断电装置随时可触达，不得被遮挡或远离操作人员。
- 启动前检查机器人本体、供电、通信线缆和固定件。机器人状态异常、吊装不可靠、网络不稳定或现场有人进入安全区域时，不得开始或继续测试。

### 10.2 实机前三级验证

必须遵循“重定向参考动作检查 → MuJoCo 仿真确认 → 实机测试”的顺序：

1. **检查重定向参考动作**：勾选「保留重定向窗口」，确认参考动作与人体动作接近，左右侧、关节方向和动作幅度正确，且没有姿态突跳、抖动或明显失真。
2. **进行仿真确认**：选择 **MuJoCo 仿真**，完整执行计划动作，检查平衡、足部接触、关节运动和动作连续性，并实际验证暂停跟踪及急停流程。
3. **进行实机测试**：仅在前两步正常后切换至 **Unitree G1 真机**。保持机器人悬挂并清空安全区域，从小幅、缓慢、短时动作开始，确认无异常后再逐步扩大测试范围。

任何阶段发现动作与人体不一致、关节反向、姿态突跳、持续抖动、足部异常或失衡趋势，应立即暂停并排查，不得带着已知异常进入下一阶段。

### 10.3 操作与异常处置

- **急停**：机器人处于控制中状态时，机器人按钮显示为红色「急停」。点击后会向 deploy 的 `keyboard_handler` 发送大写 `O` 键，触发急停。测试前应确认操作员熟悉急停位置和操作方式。
- **保持监护**：跟踪期间操作员不得离开控制台，监护人员应持续观察机器人、吊架和安全区域。不得依赖状态灯或软件限位代替人工监护。
- **发现异常**：出现非预期动作、剧烈抖动、失衡、通信中断或人员进入安全区域时，立即暂停跟踪；若机器人未停止或存在继续运动的风险，立即触发急停，必要时按现场规程切断机器人电源。待机器人完全停止并排除原因后，才能重新测试。
- **正常结束顺序**：结束任务时务必先暂停跟踪、再急停机器人、最后停止重定向，避免在机器人仍运动时切断上游数据。
- **安全参数不能替代防护**：SONIC 配置中含关节速度限制（手臂 25 rad/s、下肢 20 rad/s）、腰部俯仰角限制、关节安全模式（`kill`/`freeze`）等保护，但仍必须执行吊装、隔离和分阶段验证措施。

---

## 11. 常见问题（FAQ）

**Q1：启动界面脚本时提示没有权限或脚本未签名？**
A：在 Linux 上直接执行脚本前，可运行 `chmod +x scripts/run_teleop_studio.sh` 赋予执行权限；使用 `bash scripts/run_teleop_studio.sh` 启动则不要求脚本具有执行位。在 Windows PowerShell 中，若提示脚本未签名或禁止运行，请执行 `Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned` 调整当前用户的执行策略，再运行 `Unblock-File -Path scripts/run_teleop_studio.ps1` 解除下载文件锁定。

**Q2：启动时报 `undefined symbol _ZN14QObjectPrivateC2Ei` 之类 Qt 符号错误？**
A：这是系统 Qt6 被优先加载所致。启动脚本已自动把 PySide6 自带的 Qt6 库前置到 `LD_LIBRARY_PATH`。若仍出现，请确认是从 `scripts/run_teleop_studio.sh` 启动而非直接 `python -m teleop_studio.app`。

**Q3：网络接口下拉里没有我想要的网卡？**
A：点击右侧「刷新」重新枚举。程序会过滤掉 loopback(127.x) 与 link-local(169.254.x)，但保留 WSL 虚拟接口。若仍不可见，检查该网卡是否已分配 IPv4 地址。

**Q4：重定向启动后 GMR 输出一直卡在等待第一帧？**
A：检查 Moretion 地址 / 端口是否正确、动捕服务器是否已开始串流、协议（TCP/UDP）是否匹配（默认 TCP，需勾选 UDP 才走 UDP）。可临时用网络工具验证本机能否连到动捕服务器。

**Q5：机器人连接后控制状态灯一直橙色「等待控制」？**
A：查看 SONIC 输出日志。程序依赖日志关键字解析状态：`transitioning to control state` → 控制中，`planner initialized successfully` → 跟踪中。若 deploy.sh 提示确认未自动通过，或部署目录 / ZMQ Host / 端口配置有误，都会卡住。确认 `--zmq-host` 填的是操作机上能被 SONIC 部署主机访问到的 IP，`--zmq-port` 与 SONIC ZMQ 端口一致。

**Q6：跟踪按钮是灰色不可点？**
A：跟踪按钮仅在「机器人已连接且控制中」时可用。请先完成步骤 4，等到控制状态灯变绿。

**Q7：Windows 下没有「本地部署」选项？**
A：本地部署模式依赖 Linux PTY，Windows 仅支持远程 SSH 部署模式，该选项会被隐藏 / 禁用。

Windows 串流到 Linux 仿真时，请使用 **远程 SSH** + **MuJoCo 仿真**。远程 SSH 并不代表运行目标必须是真机。

**Q8：如何切换机器人型号？**
A：当前 SONIC 发布仅支持 G1（29 DOF）。`moretion_stream_to_sonic.py` 的 `--robot` 参数虽支持 `unitree_g1`、`unitree_g1_with_hands`、`unitree_h1_2`、`Q1`、`X1`，但界面默认面向 G1 29-DOF，切换型号需确认对应配置与关节顺序映射。

**Q9：停止重定向后 MuJoCo 窗口不消失？**
A：若勾选了「保留重定向窗口」，窗口会保留以便观察。取消勾选后，子进程退出会自动关闭窗口。

---

> 文档基于 `teleop_studio/` 源码与 `scripts/run_teleop_studio.*` 启动脚本整理。如界面或参数有调整，请同步更新本文档。
