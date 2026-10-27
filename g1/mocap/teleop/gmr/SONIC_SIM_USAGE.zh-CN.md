# SONIC MuJoCo 仿真环境包

English documentation: [README.md](README.md)

## 1. 包含内容

该压缩包提供可移植的 Ubuntu x86_64 SONIC MuJoCo 仿真环境，包含：

- Python 3.11 可重定位运行环境
- MuJoCo 仿真器与 G1 43-DoF 场景模型
- SONIC 仿真循环、CycloneDDS 0.10.2 和 Unitree DDS 消息定义
- `run_sonic_sim.sh` 启动器

压缩包不包含 `gear_sonic_deploy`、WBC 策略文件、GMR 重定向环境或真实机器人部署程序。仿真器需要与单独部署的 SONIC WBC 进程通过 CycloneDDS 通信。

## 2. 系统要求

- Ubuntu Desktop 20.04 或更高版本
- x86_64 CPU
- glibc 2.28 或更高版本
- 桌面显示模式需要可用的 X11/Wayland 和 OpenGL 驱动
- 无显示器服务器请使用 `--no-onscreen`

查看当前架构和 glibc 版本：

```bash
uname -m
getconf GNU_LIBC_VERSION
```

## 3. 校验与解压

将压缩包和对应的 `.sha256` 文件放在同一目录：

```bash
sha256sum -c SonicSim-linux-x86_64-<version>.tar.gz.sha256
mkdir -p SonicSim
tar -xzf SonicSim-linux-x86_64-<version>.tar.gz -C SonicSim
cd SonicSim
chmod +x run_sonic_sim.sh
```

请解压到一个空目录。首次运行时，启动器会自动执行一次 `conda-unpack`，修复移动后的环境路径。

## 4. 启动仿真器

Ubuntu Desktop 带 MuJoCo 窗口：

```bash
./run_sonic_sim.sh
```

无头模式：

```bash
./run_sonic_sim.sh --no-onscreen
```

显示全部参数：

```bash
./run_sonic_sim.sh --help
```

按 `Ctrl+C` 停止仿真器。

## 5. 连接 SONIC WBC

本机联调时，仿真器和 WBC 都使用回环接口：

```bash
# 终端 1：启动仿真器
./run_sonic_sim.sh --interface sim

# 终端 2：在单独安装的 gear_sonic_deploy 目录中启动 WBC
cd /path/to/gear_sonic_deploy
bash deploy.sh --input-type zmq \
  --zmq-host localhost --zmq-port 5556 --zmq-topic pose sim
```

`gear_sonic_deploy` 及其策略文件必须另行准备。没有 WBC 发布控制指令时，仿真器等待 DDS 数据属于正常现象。

跨主机联调时，为仿真器指定 WBC 主机可达的本机网卡名或 IP：

```bash
./run_sonic_sim.sh --interface enp3s0
# 或
./run_sonic_sim.sh --interface 192.168.1.20
```

仿真器与 WBC 必须使用相同的 DDS Domain ID 和可互通的网络接口。防火墙需要允许 CycloneDDS 使用的 UDP 单播/组播流量。不要给仿真器使用 `--interface real`。

## 6. 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--interface sim` | `sim` | 使用本机回环接口 |
| `--interface <网卡或IP>` | | 跨主机 DDS 通信 |
| `--sim-frequency 200` | `200` | MuJoCo 仿真频率，单位 Hz |
| `--control-frequency 50` | `50` | 控制频率，单位 Hz |
| `--no-onscreen` | 关闭 | 禁用桌面 viewer |
| `--offscreen` | 关闭 | 启用离屏渲染 |
| `--enable-image-publish` | 关闭 | 通过 ZMQ 发布相机图像，必须同时使用 `--offscreen` |
| `--camera-port 5555` | `5555` | 图像发布端口 |

## 7. 常见问题

- **提示没有显示会话**：在 Ubuntu Desktop 图形会话中运行，或添加 `--no-onscreen`。
- **找不到 DDS 对端**：检查两端接口、Domain ID、子网、防火墙和 CycloneDDS 版本。
- **glibc 版本不满足**：使用 Ubuntu 20.04 或更高版本；不要替换包内的 Python 环境。
- **模型窗口打开但机器人不动作**：确认 `gear_sonic_deploy` 已启动并进入控制状态；没有 WBC 指令时等待是正常行为。
- **首次启动较慢**：首次运行会执行 `conda-unpack`，后续启动不会重复执行。
