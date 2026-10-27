"""本地子进程管理器。

GmrProcessManager : 重定向子进程 moretion_stream_to_sonic.py (--vis 保留 MuJoCo 窗口)
LocalDeployManager: 本地部署 SONIC 的子进程（PTY，可注入 ]/Enter/O 按键）

两者均通过 Qt 信号向 UI 回报状态与日志。
LocalDeployManager 与 ssh_manager.SshDeployManager 信号/方法同构，
故可被 MainWindow.set_ssh_manager 复用，实现"远程/本地"两种部署模式无缝切换。
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
from typing import Optional

from PySide6.QtCore import QObject, Signal, Slot


class GmrProcessManager(QObject):
    """moretion_stream_to_sonic.py 重定向子进程（非交互，无需 PTY）。

    使用 subprocess.Popen 启动子进程（而非 QProcess），以便在 Windows 上
    用 CREATE_NEW_PROCESS_GROUP + GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT)
    实现 graceful shutdown（与 Linux 的 SIGTERM 行为一致）。
    stdout/stderr 通过后台线程读取，经 Qt 信号转发到 UI。
    """

    log_line = Signal(str)
    started = Signal()
    finished = Signal()

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._proc: Optional[subprocess.Popen] = None
        self._reader: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def start(self, config) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self.log_line.emit("[warn] 重定向进程已在运行")
            return
        argv = config.moretion_command()
        argv.append("--reference_control_stdin")
        cwd = config.gmr_root or None

        kwargs = dict(
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.PIPE,
            cwd=cwd,
        )
        # Windows: CREATE_NEW_PROCESS_GROUP 使 GenerateConsoleCtrlEvent 可用
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

        try:
            self._proc = subprocess.Popen(argv, **kwargs)
        except Exception as e:
            self.log_line.emit(f"[error] 启动重定向进程失败: {e}")
            self._proc = None
            return

        self.log_line.emit(f"[cmd] {' '.join(argv)}")
        self._stop_event.clear()
        self.started.emit()

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def update_reference_settings(self, config) -> bool:
        """Update only the SONIC reference window, leaving IK and its viewer running."""
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return True  # The next start reads the current GUI configuration.
        settings = {
            "model_profile": config.model_profile,
            "preview_delay_ms": config.preview_delay_ms if config.preview_delay_ms >= 0 else None,
        }
        try:
            proc.stdin.write((json.dumps(settings) + "\n").encode("utf-8"))
            proc.stdin.flush()
        except (OSError, ValueError, AttributeError) as e:
            self.log_line.emit(f"[error] 无法同步 SONIC 预览参数，请重新启动重定向: {e}")
            return False
        return True

    def stop(self) -> None:
        if self._proc is None or self._proc.poll() is not None:
            return
        try:
            if os.name == "nt":
                # Windows: 发 CTRL_BREAK_EVENT 触发子进程 SIGBREAK handler → graceful shutdown
                import ctypes
                pid = self._proc.pid
                CTRL_BREAK_EVENT = 1
                ctypes.windll.kernel32.GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, pid)
            else:
                # Linux: 发 SIGTERM，子进程 signal_handler 设 g_running=False
                self._proc.send_signal(signal.SIGTERM)

            # 超时保底：3 秒后若仍未退出则 kill
            timer = threading.Timer(3.0, self._force_kill)
            timer.daemon = True
            timer.start()
        except Exception:
            pass

    def _force_kill(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.kill()
            self.log_line.emit("[warn] graceful stop 超时，已 kill()")

    def _read_loop(self) -> None:
        """后台线程：检测进程退出，drain stdout/stderr。

        实际的 stdout/stderr 读取由独立的 reader 线程完成，
        本线程只负责检测进程退出并触发 finished 信号。
        """
        proc = self._proc
        if proc is None:
            return

        # 启动两个独立线程读 stdout/stderr（避免 selectors 与 zmq socket 冲突）
        def _pipe_reader(pipe, prefix=""):
            try:
                for line in iter(pipe.readline, b""):
                    text = line.decode("utf-8", "replace").rstrip("\n\r")
                    if text:
                        self.log_line.emit(f"{prefix}{text}" if prefix else text)
            except Exception:
                pass
            finally:
                try:
                    pipe.close()
                except Exception:
                    pass

        t_out = threading.Thread(
            target=_pipe_reader, args=(proc.stdout,), daemon=True
        )
        t_err = threading.Thread(
            target=_pipe_reader, args=(proc.stderr, "[stderr] "), daemon=True
        )
        t_out.start()
        t_err.start()

        # 等待进程退出
        proc.wait()
        proc.stdin.close()
        # 等待 reader 线程 drain 完剩余输出
        t_out.join(timeout=2)
        t_err.join(timeout=2)

        exit_code = proc.returncode if proc.returncode is not None else -1
        self.log_line.emit(f"[exit] code={exit_code}")
        self.finished.emit()
        self._proc = None


# ===========================================================================
# 部署日志解析（远程/本地共用）
# ===========================================================================
# 关键词来自 g1_deploy_onnx_ref.cpp：
#   "[Control] DEBUG: operator_state.start=true, transitioning to CONTROL state"
#   "Stopping control system."
#   "Proceed with deployment?"  (deploy.sh 的 read -p 提示)
#   "ZMQ STREAMING MODE: ENABLED" / "ZMQ STREAMING MODE: DISABLED"
def parse_deploy_line(line: str, emit_state, emit_tracking, send_answer) -> None:
    low = line.lower()
    if "transitioning to control state" in low:
        emit_state("control")
    elif "stopping control system" in low:
        emit_state("wait")
    if "proceed with deployment" in low:
        # 应答 deploy.sh 确认提示，并自动发送 ] 进入控制系统
        # （] 保留在 PTY 输入缓冲中，待 keyboard_handler 就绪后读取）
        send_answer(b"y\n]")
    if "zmq streaming mode: enabled" in low:
        emit_tracking(True)
    elif (
        "zmq streaming mode: disabled" in low
        or "zmq streaming mode: force disabled" in low
    ):
        emit_tracking(False)
    elif "planner initialized successfully" in low:
        emit_tracking(True)
    elif "disabling planner" in low:
        emit_tracking(False)


class LocalDeployManager(QObject):
    """本地部署 SONIC 的子进程（Linux PTY，可注入按键）。

    仅支持 Linux：deploy 的 keyboard_handler 需要 stdin 是 TTY（tcgetattr），
    因此用 pty.openpty() 提供伪终端。Windows 暂不支持本地部署模式。
    信号/方法与 SshDeployManager 同构。
    """

    log_line = Signal(str)
    connected = Signal()
    disconnected = Signal()
    control_state_changed = Signal(str)
    tracking_changed = Signal(object)  # Optional[bool]

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._master: Optional[int] = None
        self._proc: Optional[subprocess.Popen] = None
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def connect(self, config) -> None:  # 启动本地 deploy
        import platform

        if platform.system() != "Linux":
            self.log_line.emit("[error] 本地部署模式仅支持 Linux（需 PTY）")
            self.disconnected.emit()
            return
        # 若残留已退出的旧进程，先清理
        if self._proc is not None and self._proc.poll() is not None:
            self._cleanup()
        if self._proc is not None:
            self.log_line.emit("[warn] deploy 进程已在运行")
            return
        cmd = config.deploy_command()  # "cd <dir> && bash deploy.sh ... sim|real"
        import pty  # Unix-only，延迟到使用处导入；Windows 本地部署不支持
        try:
            master, slave = pty.openpty()
            self._master = master
            self._proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdin=slave,
                stdout=slave,
                stderr=slave,
                close_fds=True,
                start_new_session=True,
                env=dict(os.environ),
            )
            os.close(slave)
        except Exception as e:
            self.log_line.emit(f"[error] 启动本地 deploy 失败: {e}")
            self._master = None
            self._proc = None
            self.disconnected.emit()
            return
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.connected.emit()
        self.log_line.emit(f"[cmd] {cmd}")

    def disconnect(self) -> None:
        self._cleanup()

    def send_enter(self) -> None:
        self._write(b"\n")

    def send_key(self, ch: str) -> None:
        self._write(ch.encode("utf-8"))

    def emergency_stop(self) -> None:
        self._write(b"O")  # keyboard_handler 期望大写 O

    def _write(self, data: bytes) -> None:
        if self._master is None:
            return
        try:
            os.write(self._master, data)
        except OSError:
            pass

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = os.read(self._master, 4096)
            except OSError:
                break
            if not data:
                break
            text = data.decode("utf-8", "replace")
            for line in text.splitlines():
                line = line.rstrip("\r")
                self.log_line.emit(line)
                parse_deploy_line(
                    line,
                    self.control_state_changed.emit,
                    self.tracking_changed.emit,
                    self._write,
                )
        # 进程已退出/EOF：清理残留资源，避免下次 connect 报"已在运行"
        if self._proc is not None:
            try:
                self._proc.wait(timeout=1)
            except Exception:
                pass
            self._proc = None
        if self._master is not None:
            try:
                os.close(self._master)
            except OSError:
                pass
            self._master = None
        self.disconnected.emit()

    def _cleanup(self) -> None:
        self._stop.set()
        if self._proc is not None:
            try:
                self._proc.send_signal(signal.SIGINT)
                self._proc.wait(timeout=2)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            self._proc = None
        if self._master is not None:
            try:
                os.close(self._master)
            except OSError:
                pass
            self._master = None
