# G1-EDU Moretion / GMR / SONIC 双架构最小部署包

English deployment guide: `README.md`.

## 1. 运行边界

该包负责 SONIC 控制，可运行在 G1-EDU 机载 Orin 或 x86_64 Ubuntu 计算机上。
x86_64 上的同一个控制程序既可连接真实 G1，也可连接外部提供的 Unitree DDS 仿真器。
Moretion 解析与 GMR 重定向仍运行在操作端。

```text
Moretion 动捕服务器
  -> TCP/UDP :9999
操作机 GMR/scripts/moretion_stream_to_sonic.py
  -> GMR IK 重定向为 G1 29-DOF（IsaacLab 关节顺序）
  -> ZMQ PUB :5556，topic=pose，1280-byte header，Protocol v1
G1 控制包（机载 aarch64 或 Ubuntu x86_64）
  -> ZMQ SUB -> SONIC encoder/decoder TensorRT -> Unitree DDS
  -> 真实 G1 电机（real）或外部仿真器（x86_64 sim）
```

GMR 操作机保留：`general_motion_retargeting/`、`assets/unitree_g1/`、
`scripts/moretion_stream_to_sonic.py` 及其 Python 环境。它们不属于机载包。

通用包保留：一份 SONIC 控制程序最小源码、默认 release 与 low-latency 两套 encoder/decoder
ONNX、各自观测配置、aarch64 与 x86_64 两套 Unitree SDK2 库、ZMQ/MsgPack 构建入口，以及约
400 KB 的启动参考动作。
启动参考动作只是当前控制器构造时的必需占位；切入 ZMQ 后由实时流覆盖。

以下内容被明确排除：训练代码、Isaac Lab、外部 MuJoCo 仿真器、可视化、ROS2、planner、MotionBricks、
Decoupled WBC、数据处理、测试以及 `.trt` 缓存。TensorRT engine 与 GPU、
TensorRT 版本和模型绑定，必须在实际部署计算机上从 ONNX 生成，不能跨机器复制缓存。

## 2. 平台前提

G1-EDU 机载 aarch64：

- G1-EDU 机载 Orin，aarch64。
- JetPack 6。
- TensorRT **10.7**。仓库文档明确要求该版本；其他版本可能产生错误动作。
- 机器人 DDS 网口已经配置为 `192.168.123.x` 网段。
- 机载计算机可通过另一网口/Wi-Fi访问运行 GMR 的操作机。
- 真机首次运行时机器人必须悬吊，现场人员必须能立即硬件急停。

x86_64 Ubuntu：

- Ubuntu 20.04、22.04 或 24.04，x86_64。
- NVIDIA GPU 和兼容的 CUDA 12。
- TensorRT **10.13**；非系统路径安装时设置 `TensorRT_ROOT`。
- 使用 `real` 时，一块接入 `192.168.123.x` Unitree DDS 网段的有线网卡。
- 能访问 GMR ZMQ 发布端；GMR 与 SONIC 在同一台 x86_64 主机时使用 `localhost`。
- 使用 `sim` 时，需由外部工具提供同一 DDS domain 上的 Unitree SDK2 LowCmd/LowState
  仿真闭环；该仿真环境不打入本包。
- 首次真机运行同样必须悬吊机器人并准备硬件急停。

## 3. 生成和传输包

在完整 GR00T 仓库中执行：

```bash
cd gear_sonic_deploy
bash onboard/package_g1_edu.sh

scp dist/g1-edu-sonic-moretion.tar.gz unitree@G1_IP:/home/unitree/
```

只生成一个 `g1-edu-sonic-moretion.tar.gz`，可用于 G1-EDU 机载 Orin 或 x86_64 Ubuntu。
源码、模型、参考动作和说明只保存一份，仅 Unitree SDK 的小部分二进制按架构各保留一套。

打包器使用文件 allowlist，并生成 `SHA256SUMS`。若 ONNX 仍是 Git LFS 指针，打包会直接失败。

在任一目标计算机上解压同一个通用归档并执行：

```bash
cd /home/unitree
tar -xzf g1-edu-sonic-moretion.tar.gz
cd g1-edu-sonic-moretion
sha256sum -c SHA256SUMS
./install_onboard_deps.sh
./check_onboard.sh
./build_onboard.sh
```

脚本通过 `uname -m` 自动识别平台。`install_onboard_deps.sh` 只安装 C++/ZMQ 依赖，并下载与当前
平台匹配的 ONNX Runtime 1.16.3；
它不会安装或改写 JetPack、CUDA 或 TensorRT。`check_onboard.sh` 会要求 aarch64 使用 TensorRT 10.7，
x86_64 使用 TensorRT 10.13，并验证当前主机属于支持的平台。

## 4. 选择仿真或真机

`deploy.sh` 最后的 `sim/real` 是运行目标，不是编译目标。两种模式执行的是同一个
`g1_deploy_onnx_ref`：

```bash
# 外部仿真器需已在回环 DDS 接口运行
./deploy.sh --input-type zmq --zmq-host localhost --zmq-port 5556 sim

# 真实 G1，自动查找 192.168.123.x 网口
./deploy.sh --input-type zmq --zmq-host 192.168.3.10 --zmq-port 5556 real
```

`sim` 自动选择 `lo`，并添加 `--disable-crc-check`，因为外部仿真器不产生真机 CRC；
`real` 自动选择 G1 网口，并且绝不会添加该参数。本包不启动、不安装、也不限定外部
仿真环境，只提供 DDS 闭环中的 SONIC 控制端。仿真模式仅在 x86_64 上开放；aarch64
路径仍专用于 G1-EDU 机载真机控制。

## 5. 启动遥操作

先在操作机启动 GMR 发布端，确保 `--sonic_host '*'`，并允许机载计算机访问 TCP 5556：

```bash
python scripts/moretion_stream_to_sonic.py \
  --server_ip MORETION_IP --port 9999 \
  --sonic_host '*' --sonic_port 5556
```

发送端必须与部署模型匹配：`release` 对应 `--model_profile default`，
`low_latency` 对应 `--model_profile low_latency`。省略 `--preview_delay_ms` 时
保留完整真实预览（50 Hz 下为 920 / 200 ms）；Teleop Studio 默认使用折中
预览（400 / 100 ms）。每包包含 `live_frame_index`、1 帧历史、目标帧及模型
所需参考窗口，部署端按目标帧对齐，消除启动积压。`deploy.sh` 可加
`--zmq-conflate` 消费最新 pose 消息，Teleop Studio 已默认添加。
需要从本包重新编译控制器才支持目标帧对齐，仅更新 Python 发送端不够。

再在机载计算机的交互式终端启动，其中第一个参数是机器人 DDS 网口，第二个参数是操作机 IP：

```bash
./run_onboard.sh eth0 192.168.3.10 5556 release
# 或选择低延迟模型：
./run_onboard.sh eth0 192.168.3.10 5556 low_latency
```

使用 GMR Teleop Studio 时，把“部署目录”改成机载解压目录，例如
`/home/unitree/g1-edu-sonic-moretion`，运行目标选择“Unitree G1 真机”。界面生成的
`deploy.sh --input-type zmq ... real` 命令由包内兼容入口处理；它会自动查找
`192.168.123.x` 的 DDS 网口，也可在机载 shell 中设置 `ROBOT_INTERFACE=eth0` 明确指定。
界面的“默认模型”使用 `policy/release`；选择“低延迟模型”后，Teleop Studio 传入的
`--cp policy/low_latency/model` 会切换到 `policy/low_latency`，不会再被忽略。

在 x86_64 Ubuntu 上，外部仿真器运行后可把运行目标改为“MuJoCo 仿真”；本地部署和
远程 SSH 部署都使用相同的 `deploy.sh ... sim` 路径。选择“Unitree G1 真机”时则使用
`deploy.sh ... real`。

控制程序启动后的人工顺序：

1. 保持 G1 悬吊，等待 LowState、encoder 和 decoder 初始化完成。
2. 按 `]`，从等待态进入电机控制态。
3. 确认默认姿态和电机状态正常后按 `Enter`，切入 ZMQ 实时动作。
4. 异常时按 `O` 触发软件急停，同时准备使用硬件急停。

Teleop Studio 通过 SSH PTY 执行相同的 `]`、`Enter`、`O` 注入。包内 `deploy.sh` 保留了界面
依赖的 `Proceed with deployment?` 提示，因此自动确认和按键注入仍然生效。注意当前 GMR 发布端虽然也发送
`command` topic，但本包为保持最小闭包使用 `--input-type zmq`，该模式只订阅 `pose`；因此控制态和
跟踪态仍由上述按键或 Teleop Studio 管理，不能依赖 `command` 消息自动启动。

x86_64 Ubuntu 上如果 Teleop Studio、GMR 和 SONIC 位于同一台计算机，可选择“本地部署”，
部署目录指向解压后的通用包，ZMQ Host 使用 `localhost`。运行目标按需选择“MuJoCo 仿真”
或“Unitree G1 真机”；也可通过 SSH 部署到另一台 x86_64 控制计算机。

## 6. 网络与故障定位

- `robot_network_interface` 只用于 Unitree DDS，通常是 `192.168.123.x` 的有线网卡。
- 仿真使用回环接口 `lo`；启动控制器前先启动外部 DDS 仿真器。
- `gmr_operator_ip` 是操作机对机载计算机可达的地址，不是机器人 DDS 地址。
- ZMQ 是机载 SUB 主动连接操作机 PUB，所以检查方向应是从机载端到操作机 `IP:5556`。
- 首次 ONNX 转 TensorRT engine 会明显慢于后续启动；不要在转换期间进入控制态。
- 不要启用 `--disable-crc-check`，该选项仅供 MuJoCo 仿真使用。
- 不要把开发机或其他 TensorRT 版本生成的 `.trt` 文件放入 `policy/`。
