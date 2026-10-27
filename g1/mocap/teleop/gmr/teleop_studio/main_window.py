#!/usr/bin/env python3
"""Teleop Studio 主窗口 UI.

moretion -> GMR -> SONIC 遥操作的可视化控制面板。

职责边界：
  * 本文件只负责 UI 布局、状态灯、控制按钮与日志展示。
  * 本地重定向子进程 / SSH PTY 管理由外部 manager 注入：
      set_gmr_manager(process_manager)
      set_ssh_manager(ssh_manager)
    manager 的信号约定见 set_gmr_manager / set_ssh_manager。
  * 未注入 manager 时，按钮点击会在状态栏给出提示，UI 仍可预览。

跨平台：Ubuntu 22.04+ / Windows 11。网络接口枚举优先用 net_helper，
不可用时回退到最小内联实现，避免依赖 sonic_sim 包（其 __init__ 会加载
Linux-only 的 libddsc.so）。
"""

from __future__ import annotations

import os
import platform
import re
import shlex
import subprocess
import sys
from dataclasses import dataclass
from typing import Optional

from PySide6.QtCore import QSettings, Qt, Signal, Slot
from PySide6.QtGui import QAction, QActionGroup, QFont, QKeyEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

try:
    from .i18n import DEFAULT_LANGUAGE, SUPPORTED_LANGUAGES, translate
except ImportError:  # Support the documented direct preview entry point.
    from i18n import DEFAULT_LANGUAGE, SUPPORTED_LANGUAGES, translate


# ---------------------------------------------------------------------------
# 跨平台按钮：Space / Enter / Return 均触发 click()
# ---------------------------------------------------------------------------
class CrossPlatformButton(QPushButton):
    """QPushButton 子类，统一 Linux/Windows 键盘行为。

    Qt 在 Windows 上对非默认按钮的 Enter/Return 不触发 clicked()，
    且 setStyleSheet() 后可能丢失焦点导致 Space 也失效。
    本类显式捕获 Space/Enter/Return，保证跨平台一致。
    """

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter):
            self.click()
            event.accept()
        else:
            super().keyPressEvent(event)


# ---------------------------------------------------------------------------
# 跨平台网络接口枚举
# ---------------------------------------------------------------------------
def list_network_interfaces() -> dict:
    """返回 {interface_name: [ip, ...]}，包含所有 IPv4 接口（含 WSL）。

    优先级：psutil > ipconfig/ip 正则解析。
    仅过滤 loopback (127.x) 和 link-local (169.254.x)，保留 WSL 等虚拟接口供用户选择。
    """
    # --- 优先：psutil（跨平台、无编码问题）---
    try:
        import psutil

        interfaces: dict = {}
        for name, addrs in psutil.net_if_addrs().items():
            # 跳过 loopback 伪接口
            low = name.lower()
            if "loopback" in low:
                continue
            for addr in addrs:
                if addr.family.name == "AF_INET":
                    ip = addr.address
                    # 跳过 loopback 和 link-local
                    if ip.startswith("127.") or ip.startswith("169.254."):
                        continue
                    interfaces.setdefault(name, []).append(ip)
        if interfaces:
            return interfaces
    except Exception:
        pass

    # --- 回退：ipconfig / ip addr 正则解析 ---
    try:
        if platform.system() == "Windows":
            r = subprocess.run(["ipconfig"], capture_output=True)
            out = r.stdout.decode("mbcs", errors="replace")
            cur = None
            for line in out.split("\n"):
                # 匹配适配器标题行，如 "以太网适配器 以太网:" 或 "Wireless LAN adapter WLAN:"
                m = re.match(r"^(.+?)适配器\s+(.+?):$|^(?:.+?\s+)?adapter\s+(.+?):$", line.strip())
                if m:
                    cur_name = m.group(2) or m.group(3)
                    if cur_name:
                        cur_name = cur_name.strip()
                        cur = cur_name
                        interfaces.setdefault(cur, [])
                    continue
                if cur:
                    ip = re.search(r"IPv4[^\d]*:\s*(\d+\.\d+\.\d+\.\d+)", line)
                    if ip:
                        ip_val = ip.group(1)
                        if not ip_val.startswith("127.") and not ip_val.startswith("169.254."):
                            interfaces[cur].append(ip_val)
        else:
            r = subprocess.run(
                ["/sbin/ip", "addr", "show"], capture_output=True, text=True
            )
            cur = None
            for line in r.stdout.split("\n"):
                m = re.match(r"^\d+:\s+(\w+):", line)
                if m:
                    cur = m.group(1)
                    interfaces.setdefault(cur, [])
                elif cur:
                    ip = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)", line)
                    if ip:
                        ip_val = ip.group(1)
                        if not ip_val.startswith("127.") and not ip_val.startswith("169.254."):
                            interfaces[cur].append(ip_val)
    except Exception:
        pass
    return interfaces


# ---------------------------------------------------------------------------
# 运行参数
# ---------------------------------------------------------------------------
@dataclass
class TeleopConfig:
    conda_python: str = sys.executable
    gmr_root: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    moretion_ip: str = "192.168.2.132"
    moretion_port: int = 9999
    sonic_port: int = 5556
    use_vis: bool = True
    use_udp: bool = False
    zmq_host: str = ""  # 给 deploy 的 --zmq-host（本机 WiFi IP）
    ssh_host: str = "192.168.3.9"
    ssh_user: str = "unitree"
    ssh_password: str = ""
    ssh_key: str = ""
    deploy_dir: str = (
        "/home/unitree/workspace/GR00T-WholeBodyControl/gear_sonic_deploy"
    )
    input_type: str = "zmq"
    zmq_topic: str = "pose"
    model_profile: str = "default"
    checkpoint: str = "policy/low_latency/model"
    observation_config: str = "policy/low_latency/observation_config.yaml"
    deploy_target: str = "sim"
    zmq_conflate: bool = True
    preview_delay_ms: int = -1  # Auto: preserve the selected encoder's complete lookahead.

    def moretion_command(self) -> list:
        """构造 moretion_stream_to_sonic.py 的 argv。"""
        script = os.path.join(self.gmr_root, "scripts", "moretion_stream_to_sonic.py")
        argv = [
            self.conda_python or "python",
            script,
            "--server_ip",
            self.moretion_ip,
            "--port",
            str(self.moretion_port),
            "--sonic_host",
            "*",
            "--sonic_port",
            str(self.sonic_port),
            "--reference_mode",
            "preview",
            "--model_profile",
            self.model_profile,
        ]
        if self.preview_delay_ms >= 0:
            argv.extend(["--preview_delay_ms", str(self.preview_delay_ms)])
        if self.use_udp:
            argv.append("--use_udp")
        if self.use_vis:
            argv.append("--vis")
        return argv

    def deploy_command(self) -> str:
        """构造在本地或远程部署主机执行的 deploy 命令字符串。"""
        quote = shlex.quote
        command = (
            f"\ncd {quote(self.deploy_dir)} && bash deploy.sh "
        )
        if self.model_profile == "low_latency":
            command += (
                f"--cp {quote(self.checkpoint)} "
                f"--obs-config {quote(self.observation_config)} "
            )
        command += (
            f"--input-type {quote(self.input_type)} "
            f"--zmq-host {quote(self.zmq_host)} "
            f"--zmq-port {self.sonic_port} "
            f"--zmq-topic {quote(self.zmq_topic)}"
        )
        if self.zmq_conflate:
            command += " --zmq-conflate"
        return f"{command} {quote(self.deploy_target)}"


# ---------------------------------------------------------------------------
# 状态指示灯
# ---------------------------------------------------------------------------
class StatusLed(QWidget):
    """彩色圆点 + 标题 + 状态文字 的指示灯。"""

    COLORS = {
        "idle": "#9ca3af",
        "ok": "#22c55e",
        "warn": "#f59e0b",
        "error": "#ef4444",
        "active": "#3b82f6",
    }

    def __init__(self, title: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._dot = QLabel()
        self._dot.setFixedSize(16, 16)
        self._title = QLabel(title)
        self._title.setStyleSheet("font-weight:600;")
        self._state = QLabel()
        self._state.setMinimumWidth(135)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(8)
        lay.addWidget(self._dot)
        lay.addWidget(self._title)
        lay.addWidget(self._state, 1)
        self.set_state("idle", "")

    def set_state(self, level: str, text: str) -> None:
        color = self.COLORS.get(level, self.COLORS["idle"])
        self._dot.setStyleSheet(
            f"background-color:{color}; border-radius:8px; "
            f"min-width:16px; min-height:16px; max-width:16px; max-height:16px;"
        )
        self._state.setText(text)

    def set_title(self, title: str) -> None:
        self._title.setText(title)


# ===========================================================================
# 主窗口
# ===========================================================================
class MainWindow(QMainWindow):
    """Teleop Studio 主窗口。"""

    # ---- 对外信号（供外部控制器订阅）----
    request_start_gmr = Signal()
    request_stop_gmr = Signal()
    request_connect_robot = Signal()
    request_emergency_stop = Signal()
    request_toggle_tracking = Signal()
    reference_settings_changed = Signal()
    request_refresh_interfaces = Signal()
    deploy_mode_changed = Signal(str)  # "remote" / "local"
    language_changed = Signal(str)

    # 按钮配色方案
    _STYLES = {
        "green": (
            "QPushButton{background-color:#22c55e;color:white;font-weight:600;"
            "padding:8px;border-radius:4px;}"
            "QPushButton:hover{background-color:#16a34a;}"
            "QPushButton:disabled{background-color:#9ca3af;color:white;}"
        ),
        "red": (
            "QPushButton{background-color:#ef4444;color:white;font-weight:600;"
            "padding:8px;border-radius:4px;}"
            "QPushButton:hover{background-color:#dc2626;}"
            "QPushButton:disabled{background-color:#9ca3af;color:white;}"
        ),
        "gray": (
            "QPushButton{background-color:#6b7280;color:white;font-weight:600;"
            "padding:8px;border-radius:4px;}"
            "QPushButton:disabled{background-color:#9ca3af;color:white;}"
        ),
    }

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._settings = QSettings("GMR", "Teleop Studio")
        saved_language = str(self._settings.value("language", DEFAULT_LANGUAGE))
        self._language = (
            saved_language if saved_language in SUPPORTED_LANGUAGES else DEFAULT_LANGUAGE
        )
        self.resize(1100, 760)

        # 运行时状态
        self._gmr_manager = None
        self._ssh_manager = None
        self._gmr_running = False
        self._ssh_connected = False
        self._control_active = False
        self._control_state = "unknown"
        self._tracking_on: Optional[bool] = None  # None=未知
        self._gmr_transition = False  # 重定向过渡态
        self._robot_transition = False  # 机器人过渡态

        self._build_language_menu()

        self._central = QWidget()
        self.setCentralWidget(self._central)
        root = QVBoxLayout(self._central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        root.addWidget(self._build_param_group())
        root.addWidget(self._build_status_group())
        root.addWidget(self._build_control_group())
        root.addWidget(self._build_log_group(), 1)

        # 自动确定路径（开发态与打包态均成立）：
        #   conda_python = 当前解释器（启动器从打包 env 或 conda 激活后运行即正确）
        #   gmr_root     = 由本文件位置上溯（teleop_studio/ -> 仓库根）
        self._conda_python_edit.setText(sys.executable)
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self._gmr_root_edit.setText(repo_root)

        self._retranslate_ui()
        self._refresh_interfaces()
        self._apply_deploy_mode(self.deploy_mode())
        self._sync_buttons()
        self.statusBar().showMessage(self.text("ready"), 4000)

    def text(self, key: str, **kwargs) -> str:
        """Return a translated string in the currently selected language."""
        return translate(self._language, key, **kwargs)

    def language(self) -> str:
        return self._language

    def _build_language_menu(self) -> None:
        self._language_menu = self.menuBar().addMenu("")
        self._language_actions = QActionGroup(self)
        self._language_actions.setExclusive(True)
        for label, language in (("中文", "zh_CN"), ("English", "en")):
            action = QAction(label, self, checkable=True)
            action.setData(language)
            action.setChecked(language == self._language)
            action.triggered.connect(
                lambda checked, code=language: checked and self.set_language(code)
            )
            self._language_actions.addAction(action)
            self._language_menu.addAction(action)

    @Slot(str)
    def set_language(self, language: str) -> None:
        if language not in SUPPORTED_LANGUAGES or language == self._language:
            return
        self._language = language
        self._settings.setValue("language", language)
        for action in self._language_actions.actions():
            action.setChecked(action.data() == language)
        self._retranslate_ui()
        self.language_changed.emit(language)
        self.flash(self.text("language_changed"))

    def _retranslate_ui(self) -> None:
        """Update visible text without rebuilding widgets or losing form state."""
        self.setWindowTitle(self.text("window_title"))
        self._language_menu.setTitle(self.text("menu_language"))

        self._param_group.setTitle(self.text("group_parameters"))
        self._status_group.setTitle(self.text("group_status"))
        self._control_group.setTitle(self.text("group_control"))
        self._log_group.setTitle(self.text("group_log"))
        for key, label in self._param_labels.items():
            label.setText(self.text(key))

        self._conda_python_edit.setPlaceholderText(self.text("python_placeholder"))
        self._ssh_pwd_edit.setPlaceholderText(self.text("password_placeholder"))
        self._ssh_key_edit.setPlaceholderText(self.text("ssh_key_placeholder"))
        self._preview_delay_spin.setSpecialValueText(self.text("preview_auto"))
        self._preview_delay_spin.setToolTip(self.text("preview_help"))
        for button in (
            self._browse_python_button,
            self._browse_root_button,
            self._browse_key_button,
        ):
            button.setText(self.text("browse"))
        self._iface_refresh_button.setText(self.text("refresh"))

        self._set_combo_item_text(self._deploy_mode_combo, "remote", "remote_deploy")
        self._set_combo_item_text(self._deploy_mode_combo, "local", "local_deploy")
        self._set_combo_item_text(self._deploy_target_combo, "sim", "simulation_target")
        self._set_combo_item_text(self._deploy_target_combo, "real", "robot_target")
        self._set_combo_item_text(self._model_profile_combo, "default", "default_model")
        self._set_combo_item_text(
            self._model_profile_combo, "low_latency", "low_latency_model"
        )
        for index in range(self._iface_combo.count()):
            if self._iface_combo.itemData(index) == ("", ""):
                self._iface_combo.setItemText(index, self.text("no_interfaces"))

        self.led_gmr.set_title(self.text("gmr_retargeting"))
        self.led_control.set_title(self.text("control_status"))
        self.led_tracking.set_title(self.text("tracking_status"))
        self._retargeting_label.setText(self.text("retargeting"))
        self._robot_label.setText(self.text("robot"))
        self._gmr_output_label.setText(self.text("gmr_output"))
        self._sonic_output_label.setText(self.text("sonic_output"))

        self._apply_deploy_mode(self.deploy_mode())
        self._sync_status_texts()
        self._sync_buttons()

    def _set_combo_item_text(self, combo: QComboBox, data, text_key: str) -> None:
        index = combo.findData(data)
        if index >= 0:
            combo.setItemText(index, self.text(text_key))

    def _sync_status_texts(self) -> None:
        self.led_gmr.set_state(
            "active" if self._gmr_running else "idle",
            self.text("running" if self._gmr_running else "stopped"),
        )
        self.led_ssh.set_state(
            "ok" if self._ssh_connected else "idle",
            self.text("connected" if self._ssh_connected else "not_connected"),
        )
        if self._control_state == "wait":
            self.led_control.set_state("warn", self.text("waiting_for_control"))
        elif self._control_state == "control":
            self.led_control.set_state("ok", self.text("controlling"))
        else:
            self.led_control.set_state("idle", self.text("unknown"))
        self._set_tracking_led(self._tracking_on)

    # ------------------------------------------------------------------
    # 参数表单
    # ------------------------------------------------------------------
    def _build_param_group(self) -> QGroupBox:
        self._param_group = QGroupBox()
        g = QGridLayout(self._param_group)
        g.setColumnStretch(1, 1)
        self._param_labels = {}

        def add_label(key: str, row: int) -> None:
            label = QLabel()
            self._param_labels[key] = label
            g.addWidget(label, row, 0)

        # --- 重定向 (GMR) ---
        add_label("gmr_python", 0)
        self._conda_python_edit = QLineEdit()
        self._browse_python_button = QPushButton()
        self._browse_python_button.clicked.connect(self._browse_conda_python)
        g.addWidget(self._conda_python_edit, 0, 1)
        g.addWidget(self._browse_python_button, 0, 2)

        add_label("gmr_root", 1)
        self._gmr_root_edit = QLineEdit("/home/zpf/workspace/GMR")
        self._browse_root_button = QPushButton()
        self._browse_root_button.clicked.connect(self._browse_gmr_root)
        g.addWidget(self._gmr_root_edit, 1, 1)
        g.addWidget(self._browse_root_button, 1, 2)

        add_label("moretion_address", 2)
        self._moretion_ip_edit = QLineEdit("192.168.2.132")
        g.addWidget(self._moretion_ip_edit, 2, 1)

        add_label("moretion_port", 3)
        self._moretion_port_spin = QSpinBox()
        self._moretion_port_spin.setRange(1, 65535)
        self._moretion_port_spin.setValue(9999)
        g.addWidget(self._moretion_port_spin, 3, 1)

        add_label("udp", 4)
        self._udp_check = QCheckBox("")
        self._udp_check.setChecked(False)
        g.addWidget(self._udp_check, 4, 1)

        add_label("keep_retarget_window", 5)
        self._vis_check = QCheckBox("")
        self._vis_check.setChecked(True)
        g.addWidget(self._vis_check, 5, 1)

        # --- 网络接口 ---
        add_label("network_interface", 6)
        iface_row = QHBoxLayout()
        self._iface_combo = QComboBox()
        self._iface_combo.currentIndexChanged.connect(self._on_iface_changed)
        self._iface_refresh_button = QPushButton()
        self._iface_refresh_button.clicked.connect(self._refresh_interfaces)
        iface_row.addWidget(self._iface_combo, 1)
        iface_row.addWidget(self._iface_refresh_button)
        iface_container = QWidget()
        iface_container.setLayout(iface_row)
        g.addWidget(iface_container, 6, 1)

        add_label("sonic_zmq_host", 7)
        self._zmq_host_label = QLabel()
        self._zmq_host_label.setStyleSheet("color:#3b82f6; font-weight:600;")
        g.addWidget(self._zmq_host_label, 7, 1)

        add_label("sonic_zmq_port", 8)
        self._sonic_port_spin = QSpinBox()
        self._sonic_port_spin.setRange(1, 65535)
        self._sonic_port_spin.setValue(5556)
        g.addWidget(self._sonic_port_spin, 8, 1)

        # --- 部署模式 ---
        add_label("deploy_mode", 9)
        self._deploy_mode_combo = QComboBox()
        self._deploy_mode_combo.addItem("", "remote")
        if platform.system() != "Windows":
            self._deploy_mode_combo.addItem("", "local")
        self._deploy_mode_combo.currentIndexChanged.connect(self._on_deploy_mode_combo)
        g.addWidget(self._deploy_mode_combo, 9, 1)

        add_label("deploy_target", 10)
        self._deploy_target_combo = QComboBox()
        self._deploy_target_combo.addItem("", "sim")
        self._deploy_target_combo.addItem("", "real")
        g.addWidget(self._deploy_target_combo, 10, 1)

        add_label("control_model", 11)
        self._model_profile_combo = QComboBox()
        self._model_profile_combo.addItem("", "default")
        self._model_profile_combo.addItem("", "low_latency")
        g.addWidget(self._model_profile_combo, 11, 1)

        add_label("preview_delay", 12)
        self._preview_delay_spin = QSpinBox()
        self._preview_delay_spin.setRange(-1, 2000)
        self._preview_delay_spin.setValue(400)
        self._preview_delay_spin.setSuffix(" ms")
        g.addWidget(self._preview_delay_spin, 12, 1)
        self._model_profile_combo.currentIndexChanged.connect(self._on_model_profile_changed)
        self._preview_delay_spin.valueChanged.connect(self.reference_settings_changed)

        # --- SSH / 机器人部署 ---
        add_label("ssh_host", 13)
        self._ssh_host_edit = QLineEdit("192.168.3.9")
        g.addWidget(self._ssh_host_edit, 13, 1)

        add_label("ssh_user", 14)
        self._ssh_user_edit = QLineEdit("unitree")
        g.addWidget(self._ssh_user_edit, 14, 1)

        add_label("ssh_password", 15)
        self._ssh_pwd_edit = QLineEdit()
        self._ssh_pwd_edit.setEchoMode(QLineEdit.Password)
        g.addWidget(self._ssh_pwd_edit, 15, 1)

        add_label("ssh_key", 16)
        key_row = QHBoxLayout()
        self._ssh_key_edit = QLineEdit()
        self._browse_key_button = QPushButton()
        self._browse_key_button.clicked.connect(self._browse_ssh_key)
        key_row.addWidget(self._ssh_key_edit, 1)
        key_row.addWidget(self._browse_key_button)
        key_container = QWidget()
        key_container.setLayout(key_row)
        g.addWidget(key_container, 16, 1)

        add_label("deploy_directory", 17)
        self._deploy_dir_edit = QLineEdit(
            "/home/unitree/workspace/GR00T-WholeBodyControl/gear_sonic_deploy"
        )
        g.addWidget(self._deploy_dir_edit, 17, 1)

        return self._param_group

    # ------------------------------------------------------------------
    # 状态灯
    # ------------------------------------------------------------------
    def _build_status_group(self) -> QGroupBox:
        self._status_group = QGroupBox()
        h = QHBoxLayout(self._status_group)
        self.led_gmr = StatusLed("")
        self.led_ssh = StatusLed("")
        self.led_control = StatusLed("")
        self.led_tracking = StatusLed("")
        for led in (self.led_gmr, self.led_ssh, self.led_control, self.led_tracking):
            h.addWidget(led)
        h.addStretch(1)
        return self._status_group

    # ------------------------------------------------------------------
    # 控制按钮
    # ------------------------------------------------------------------
    def _build_control_group(self) -> QGroupBox:
        self._control_group = QGroupBox()
        v = QVBoxLayout(self._control_group)
        v.setSpacing(12)

        # 左列：重定向（上）+ 机器人（下）；中间：跟踪按钮（倒角正方形，跨两行）
        grid = QGridLayout()
        grid.setSpacing(12)

        # 左列第 0 行：重定向
        self._retargeting_label = QLabel()
        grid.addWidget(self._retargeting_label, 0, 0, Qt.AlignRight)
        self.btn_gmr = CrossPlatformButton()
        self.btn_gmr.setMinimumWidth(140)
        self.btn_gmr.clicked.connect(self._on_gmr_button)
        grid.addWidget(self.btn_gmr, 0, 1, Qt.AlignLeft)

        # 左列第 1 行：机器人
        self._robot_label = QLabel()
        grid.addWidget(self._robot_label, 1, 0, Qt.AlignRight)
        self.btn_robot = CrossPlatformButton()
        self.btn_robot.setMinimumWidth(140)
        self.btn_robot.clicked.connect(self._on_robot_button)
        grid.addWidget(self.btn_robot, 1, 1, Qt.AlignLeft)

        # 中间：跟踪按钮（倒角正方形，跨两行）
        self.btn_toggle_tracking = CrossPlatformButton()
        self.btn_toggle_tracking.setFont(QFont("", 14))
        self.btn_toggle_tracking.setMinimumHeight(100)
        self.btn_toggle_tracking.setMinimumWidth(160)
        self.btn_toggle_tracking.setMaximumHeight(140)
        self.btn_toggle_tracking.setMaximumWidth(200)
        self.btn_toggle_tracking.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed
        )
        self.btn_toggle_tracking.clicked.connect(self._on_toggle_tracking)
        grid.addWidget(self.btn_toggle_tracking, 0, 2, 2, 1)
        grid.setAlignment(self.btn_toggle_tracking, Qt.AlignCenter)

        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 1)
        grid_w = QWidget()
        grid_w.setLayout(grid)
        v.addWidget(grid_w)

        return self._control_group

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------
    def _build_log_group(self) -> QGroupBox:
        self._log_group = QGroupBox()
        h = QHBoxLayout(self._log_group)

        left = QVBoxLayout()
        self._gmr_output_label = QLabel()
        left.addWidget(self._gmr_output_label)
        self._gmr_log = QPlainTextEdit()
        self._gmr_log.setReadOnly(True)
        self._gmr_log.setMaximumBlockCount(5000)
        self._gmr_log.setFont(QFont("Menlo,Consolas,monospace"))
        left.addWidget(self._gmr_log, 1)
        leftw = QWidget()
        leftw.setLayout(left)

        right = QVBoxLayout()
        self._sonic_output_label = QLabel()
        right.addWidget(self._sonic_output_label)
        self._deploy_log = QPlainTextEdit()
        self._deploy_log.setReadOnly(True)
        self._deploy_log.setMaximumBlockCount(5000)
        self._deploy_log.setFont(QFont("Menlo,Consolas,monospace"))
        right.addWidget(self._deploy_log, 1)
        rightw = QWidget()
        rightw.setLayout(right)

        h.addWidget(leftw, 1)
        h.addWidget(rightw, 1)
        return self._log_group

    # ==================================================================
    # 参数读取 / 文件浏览
    # ==================================================================
    def read_config(self) -> TeleopConfig:
        iface_data = self._iface_combo.currentData()
        zmq_host = "localhost" if self.deploy_mode() == "local" else ""
        if self.deploy_mode() != "local" and iface_data:
            _name, zmq_host = iface_data
        return TeleopConfig(
            conda_python=self._conda_python_edit.text().strip(),
            gmr_root=self._gmr_root_edit.text().strip(),
            moretion_ip=self._moretion_ip_edit.text().strip(),
            moretion_port=self._moretion_port_spin.value(),
            sonic_port=self._sonic_port_spin.value(),
            use_vis=self._vis_check.isChecked(),
            use_udp=self._udp_check.isChecked(),
            zmq_host=zmq_host,
            ssh_host=self._ssh_host_edit.text().strip(),
            ssh_user=self._ssh_user_edit.text().strip(),
            ssh_password=self._ssh_pwd_edit.text(),
            ssh_key=self._ssh_key_edit.text().strip(),
            deploy_dir=self._deploy_dir_edit.text().strip(),
            model_profile=self._model_profile_combo.currentData() or "default",
            deploy_target=self._deploy_target_combo.currentData() or "sim",
            preview_delay_ms=self._preview_delay_spin.value(),
        )

    def _browse_conda_python(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, self.text("select_python"), "", "python (python python.exe)"
        )
        if path:
            self._conda_python_edit.setText(path)

    def _browse_gmr_root(self) -> None:
        path = QFileDialog.getExistingDirectory(self, self.text("select_gmr_root"))
        if path:
            self._gmr_root_edit.setText(path)

    def _browse_ssh_key(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, self.text("select_ssh_key"))
        if path:
            self._ssh_key_edit.setText(path)

    # ==================================================================
    # 网络接口
    # ==================================================================
    def _refresh_interfaces(self) -> None:
        self._iface_combo.blockSignals(True)
        self._iface_combo.clear()
        ifaces = list_network_interfaces()
        # 排序：WSL/vEthernet 排到最后
        sorted_ifaces = sorted(
            ifaces.items(),
            key=lambda kv: ("wsl" in kv[0].lower() or "vethernet" in kv[0].lower(), kv[0]),
        )
        preferred_idx = 0
        for i, (name, ips) in enumerate(sorted_ifaces):
            for ip in ips:
                label = f"{name}  ({ip})"
                self._iface_combo.addItem(label, (name, ip))
                # 默认优选机器人网段
                if ip.startswith("192.168.3."):
                    preferred_idx = self._iface_combo.count() - 1
        if self._iface_combo.count() == 0:
            self._iface_combo.addItem(self.text("no_interfaces"), ("", ""))
        self._iface_combo.setCurrentIndex(preferred_idx)
        self._iface_combo.blockSignals(False)
        self._on_iface_changed(self._iface_combo.currentIndex())

    def _on_iface_changed(self, _idx: int) -> None:
        if self.deploy_mode() == "local":
            return  # 本地部署固定 localhost，忽略接口选择
        data = self._iface_combo.currentData()
        if data:
            _name, ip = data
            self._zmq_host_label.setText(ip if ip else self.text("not_selected"))
        else:
            self._zmq_host_label.setText(self.text("not_selected"))

    # ==================================================================
    # 部署模式
    # ==================================================================
    def deploy_mode(self) -> str:
        return self._deploy_mode_combo.currentData() or "remote"

    @Slot(int)
    def _on_model_profile_changed(self, _idx: int) -> None:
        delay_ms = 100 if self._model_profile_combo.currentData() == "low_latency" else 400
        self._preview_delay_spin.blockSignals(True)
        self._preview_delay_spin.setValue(delay_ms)
        self._preview_delay_spin.blockSignals(False)
        self.reference_settings_changed.emit()

    @Slot(int)
    def _on_deploy_mode_combo(self, _idx: int) -> None:
        mode = self.deploy_mode()
        self._apply_deploy_mode(mode)
        self.deploy_mode_changed.emit(mode)

    def _apply_deploy_mode(self, mode: str) -> None:
        ssh_fields = (
            self._ssh_host_edit,
            self._ssh_user_edit,
            self._ssh_pwd_edit,
            self._ssh_key_edit,
        )
        local = mode == "local"
        for w in ssh_fields:
            w.setEnabled(not local)
        self._iface_combo.setEnabled(not local)
        # 切换部署目录默认值（仅当用户未自定义时）
        remote_default = "/home/unitree/workspace/GR00T-WholeBodyControl/gear_sonic_deploy"
        local_default = "/home/zpf/workspace/GR00T-WholeBodyControl/gear_sonic_deploy"
        cur = self._deploy_dir_edit.text().strip()
        if local and cur == remote_default:
            self._deploy_dir_edit.setText(local_default)
        elif not local and cur == local_default:
            self._deploy_dir_edit.setText(remote_default)
        # zmq-host 与状态灯文案
        if local:
            self._zmq_host_label.setText("localhost")
            self.led_ssh.set_title(self.text("local_deployment"))
        else:
            self._on_iface_changed(self._iface_combo.currentIndex())
            self.led_ssh.set_title(self.text("ssh_connection"))

    # ==================================================================
    # 按钮事件
    # ==================================================================
    def _on_gmr_button(self) -> None:
        """重定向开关：未运行→启动，运行中→停止。"""
        if self._gmr_transition:
            return
        self._gmr_transition = True
        if self._gmr_running:
            self.request_stop_gmr.emit()
        else:
            self.request_start_gmr.emit()
        self._sync_buttons()

    def _on_robot_button(self) -> None:
        """机器人开关：未连接→连接（自动进入控制），控制中→急停。"""
        if self._robot_transition:
            return
        if not self._ssh_connected:
            self._robot_transition = True
            self.request_connect_robot.emit()
        elif self._control_active:
            self._robot_transition = True
            self.request_emergency_stop.emit()
        self._sync_buttons()

    def _on_toggle_tracking(self) -> None:
        self.request_toggle_tracking.emit()
        # 本地即时反馈（权威状态由 deploy 日志解析后回写）
        if self._tracking_on is None:
            self._tracking_on = True
        else:
            self._tracking_on = not self._tracking_on
        self._sync_buttons()
        self._set_tracking_led(self._tracking_on)

    # ==================================================================
    # 状态回写（供 manager 调用）
    # ==================================================================
    def set_gmr_running(self, running: bool) -> None:
        self._gmr_running = running
        self._gmr_transition = False
        self.led_gmr.set_state(
            "active" if running else "idle",
            self.text("running" if running else "stopped"),
        )
        self._sync_buttons()

    def set_ssh_connected(self, connected: bool) -> None:
        self._ssh_connected = connected
        if not connected:
            # 连接断开时，控制/跟踪态已不可知，须同步重置对应状态灯，
            # 否则状态栏会残留 “控制中 / 跟踪中” 等过期显示。
            self._robot_transition = False
            self._control_active = False
            self._control_state = "unknown"
            self._tracking_on = None
            self.led_control.set_state("idle", self.text("unknown"))
            self._set_tracking_led(None)
        self.led_ssh.set_state(
            "ok" if connected else "idle",
            self.text("connected" if connected else "not_connected"),
        )
        self._sync_buttons()

    def set_control_state(self, state: str) -> None:
        """state: 'wait' / 'control' / 'unknown'。"""
        self._robot_transition = False  # 控制状态变化，过渡完成
        self._control_state = state if state in ("wait", "control") else "unknown"
        if state == "wait":
            self._control_active = False
            self.led_control.set_state("warn", self.text("waiting_for_control"))
        elif state == "control":
            self._control_active = True
            self.led_control.set_state("ok", self.text("controlling"))
        else:
            self._control_active = False
            self.led_control.set_state("idle", self.text("unknown"))
        self._sync_buttons()

    def set_tracking_state(self, tracking: Optional[bool]) -> None:
        """由 deploy 日志解析后回写权威状态。None=未知。"""
        self._tracking_on = tracking
        self._sync_buttons()
        self._set_tracking_led(tracking)

    def _set_tracking_led(self, tracking: Optional[bool]) -> None:
        if tracking is True:
            self.led_tracking.set_state("ok", self.text("tracking"))
        elif tracking is False:
            self.led_tracking.set_state("warn", self.text("paused"))
        else:
            self.led_tracking.set_state("idle", self.text("unknown"))

    # ==================================================================
    # 日志追加
    # ==================================================================
    def append_gmr_log(self, line: str) -> None:
        self._gmr_log.appendPlainText(line.rstrip("\n"))

    def append_deploy_log(self, line: str) -> None:
        self._deploy_log.appendPlainText(line.rstrip("\n"))

    # ==================================================================
    # 按钮启用/禁用/配色联动
    # ==================================================================
    def _sync_buttons(self) -> None:
        # Sender lookahead and deployed encoder must be selected together.
        reference_editable = not (self._ssh_connected or self._robot_transition)
        self._model_profile_combo.setEnabled(reference_editable)
        self._preview_delay_spin.setEnabled(reference_editable)
        self._update_gmr_button()
        self._update_robot_button()
        self._update_tracking_button()

    def _update_gmr_button(self) -> None:
        if self._gmr_transition:
            self.btn_gmr.setText("…")
            self.btn_gmr.setStyleSheet(self._STYLES["gray"])
            self.btn_gmr.setEnabled(False)
        elif self._gmr_running:
            self.btn_gmr.setText(self.text("stop"))
            self.btn_gmr.setStyleSheet(self._STYLES["red"])
            self.btn_gmr.setEnabled(True)
        else:
            self.btn_gmr.setText(self.text("start"))
            self.btn_gmr.setStyleSheet(self._STYLES["green"])
            self.btn_gmr.setEnabled(True)

    def _update_robot_button(self) -> None:
        if self._robot_transition:
            self.btn_robot.setText("…")
            self.btn_robot.setStyleSheet(self._STYLES["gray"])
            self.btn_robot.setEnabled(False)
        elif self._ssh_connected and self._control_active:
            self.btn_robot.setText(self.text("emergency_stop"))
            self.btn_robot.setStyleSheet(self._STYLES["red"])
            self.btn_robot.setEnabled(True)
        elif self._ssh_connected:
            # 已连接但尚未进入控制 — 等待 ] 响应
            self.btn_robot.setText("…")
            self.btn_robot.setStyleSheet(self._STYLES["gray"])
            self.btn_robot.setEnabled(False)
        else:
            self.btn_robot.setText(self.text("connect"))
            self.btn_robot.setStyleSheet(self._STYLES["green"])
            self.btn_robot.setEnabled(True)

    def _update_tracking_button(self) -> None:
        # 倒角正方形样式
        track_style = (
            "QPushButton{border-radius:20px;font-weight:600;"
            "padding:8px;}"
            "QPushButton:disabled{background-color:#9ca3af;color:white;}"
        )
        if self._tracking_on is True:
            self.btn_toggle_tracking.setText(self.text("pause_tracking"))
            self.btn_toggle_tracking.setStyleSheet(
                track_style
                + "QPushButton{background-color:#ef4444;color:white;}"
                "QPushButton:hover{background-color:#dc2626;}"
            )
        else:
            self.btn_toggle_tracking.setText(self.text("start_tracking"))
            self.btn_toggle_tracking.setStyleSheet(
                track_style
                + "QPushButton{background-color:#22c55e;color:white;}"
                "QPushButton:hover{background-color:#16a34a;}"
            )
        # 仅在机器人已连接且控制中时可用
        self.btn_toggle_tracking.setEnabled(
            self._ssh_connected
            and self._control_active
            and not self._robot_transition
        )

    # ==================================================================
    # manager 注入
    # ==================================================================
    def set_gmr_manager(self, manager) -> None:
        self._gmr_manager = manager
        try:
            manager.log_line.connect(self.append_gmr_log)
            manager.started.connect(lambda: self.set_gmr_running(True))
            manager.finished.connect(lambda: self.set_gmr_running(False))
        except AttributeError:
            pass

    def set_ssh_manager(self, manager) -> None:
        self._ssh_manager = manager
        try:
            manager.log_line.connect(self.append_deploy_log)
            manager.connected.connect(lambda: self.set_ssh_connected(True))
            manager.disconnected.connect(lambda: self.set_ssh_connected(False))
            manager.control_state_changed.connect(self.set_control_state)
            manager.tracking_changed.connect(self.set_tracking_state)
        except AttributeError:
            pass

    # ==================================================================
    # 状态栏便捷提示
    # ==================================================================
    def warn(self, title: str, text: str) -> None:
        QMessageBox.warning(self, title, text)

    def flash(self, msg: str, ms: int = 3000) -> None:
        self.statusBar().showMessage(msg, ms)


# ===========================================================================
# 预览入口：直接运行本文件可查看 UI
# ===========================================================================
if __name__ == "__main__":
    import sys

    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    w = MainWindow()
    # 预览态：点亮部分状态灯
    w.set_gmr_running(True)
    w.set_ssh_connected(True)
    w.set_control_state("control")
    w.set_tracking_state(True)
    w.show()
    sys.exit(app.exec())
