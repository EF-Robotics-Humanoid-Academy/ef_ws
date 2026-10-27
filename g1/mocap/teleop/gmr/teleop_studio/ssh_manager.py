"""SSH 部署管理器：通过 paramiko invoke_shell 在 g1 上运行 deploy.sh，
并注入 ]/Enter/O 按键（与人工在终端 2 敲键等价）。

信号/方法与 process_manager.LocalDeployManager 同构：
    log_line / connected / disconnected / control_state_changed / tracking_changed
    connect / disconnect / send_enter / send_key / emergency_stop
二者可被 MainWindow.set_ssh_manager 互换，从而实现远程/本地部署模式切换。
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from PySide6.QtCore import QObject, Signal

from .process_manager import parse_deploy_line


class SshDeployManager(QObject):
    """远程 SSH 部署。paramiko 在 Ubuntu/Windows 均可用，跨平台。"""

    log_line = Signal(str)
    connected = Signal()
    disconnected = Signal()
    control_state_changed = Signal(str)
    tracking_changed = Signal(object)  # Optional[bool]

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._client = None
        self._chan = None
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def connect(self, config) -> None:  # 连接 g1 并启动 deploy.sh
        try:
            import paramiko
        except ImportError:
            self.log_line.emit("[error] 未安装 paramiko：pip install paramiko")
            self.disconnected.emit()
            return
        # 若残留已关闭的旧 channel，先清理
        if self._chan is not None:
            try:
                if self._chan.exit_status_ready():
                    self._cleanup()
            except Exception:
                self._cleanup()
        if self._chan is not None:
            self.log_line.emit("[warn] SSH 已连接")
            return
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            kwargs = dict(
                hostname=config.ssh_host, username=config.ssh_user, timeout=10
            )
            if config.ssh_key:
                kwargs["key_filename"] = config.ssh_key
            elif config.ssh_password:
                kwargs["password"] = config.ssh_password
            client.connect(**kwargs)
            self._client = client
            chan = client.invoke_shell()
            self._chan = chan
        except Exception as e:
            self.log_line.emit(f"[error] SSH 连接失败: {e}")
            self._client = None
            self._chan = None
            self.disconnected.emit()
            return
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.connected.emit()
        cmd = config.deploy_command()
        self.log_line.emit(f"[cmd] {cmd}")
        self._send_line(cmd)
        # deploy.sh 的 "Proceed with deployment?" 提示由 parse_deploy_line 自动回 y\n

    def disconnect(self) -> None:
        self._cleanup()

    def send_enter(self) -> None:
        self._write(b"\n")

    def send_key(self, ch: str) -> None:
        self._write(ch.encode("utf-8"))

    def emergency_stop(self) -> None:
        self._write(b"O")  # keyboard_handler 期望大写 O

    def _send_line(self, text: str) -> None:
        self._write((text + "\n").encode("utf-8"))

    def _write(self, data: bytes) -> None:
        if self._chan is None:
            return
        try:
            self._chan.send(data)
        except Exception:
            pass

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._chan is None:
                    break
                # 优先读取缓冲数据（即使 exit_status_ready 已触发也要 drain）
                if self._chan.recv_ready():
                    data = self._chan.recv(4096)
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
                        # 检测 deploy 进程退出（invoke_shell 模式下 shell 不会自动关闭）
                        low = line.lower()
                        if "exiting normally" in low or "program exiting" in low:
                            self._stop.set()
                            break
                    if self._stop.is_set():
                        break
                    continue  # 立即检查是否有更多数据
                # 无数据时检查 channel 是否已关闭
                if self._chan.exit_status_ready():
                    # 关闭前再 drain 一次残留数据
                    while self._chan.recv_ready():
                        data = self._chan.recv(4096)
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
                    break
                else:
                    time.sleep(0.05)
            except Exception:
                break
        # channel 已关闭/命令结束：清理残留资源
        if self._chan is not None:
            try:
                self._chan.close()
            except Exception:
                pass
            self._chan = None
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None
        self.disconnected.emit()

    def _cleanup(self) -> None:
        self._stop.set()
        if self._chan is not None:
            try:
                self._chan.close()
            except Exception:
                pass
            self._chan = None
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None
