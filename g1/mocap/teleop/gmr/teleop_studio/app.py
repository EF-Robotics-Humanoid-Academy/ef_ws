"""Teleop Studio 控制器与入口。

装配 MainWindow 与各 manager，把 UI 的 request_* 信号接到 manager 调用，
并按"部署模式"在远程 SSH / 本地部署 两种 manager 间切换。

按钮简化后，信号精简为：
  request_start_gmr / request_stop_gmr  → GmrProcessManager.start / stop
  request_connect_robot                 → deploy manager.connect（] 由 parse_deploy_line 自动发送）
  request_emergency_stop                → deploy manager.emergency_stop（发送 O）
  request_toggle_tracking               → deploy manager.send_enter（切换 ZMQ 跟踪）
"""

from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from .main_window import MainWindow, TeleopConfig
from .process_manager import GmrProcessManager, LocalDeployManager
from .ssh_manager import SshDeployManager

DEPLOY_REMOTE = "remote"  # 远程 SSH（机器人上部署 SONIC）
DEPLOY_LOCAL = "local"  # 本地部署 SONIC，有线直连机器人


class TeleopController:
    def __init__(self):
        self.window = MainWindow()
        self.gmr = GmrProcessManager(self.window)
        self.window.set_gmr_manager(self.gmr)

        self._deploy_mode: str = self.window.deploy_mode()
        self._deploy = None
        self._make_deploy_manager()

        w = self.window
        w.request_start_gmr.connect(self._on_start_gmr)
        w.request_stop_gmr.connect(self._on_stop_gmr)
        w.reference_settings_changed.connect(self._on_reference_settings_changed)
        w.request_connect_robot.connect(self._on_connect)
        w.request_emergency_stop.connect(self._on_emergency)
        w.request_toggle_tracking.connect(self._on_toggle_tracking)
        w.deploy_mode_changed.connect(self._on_deploy_mode_changed)

    # ------------------------------------------------------------------
    # manager 装配
    # ------------------------------------------------------------------
    def _make_deploy_manager(self) -> None:
        if self._deploy is not None:
            try:
                self._deploy.disconnect()
            except Exception:
                pass
        if self._deploy_mode == DEPLOY_LOCAL:
            self._deploy = LocalDeployManager(self.window)
        else:
            self._deploy = SshDeployManager(self.window)
        self.window.set_ssh_manager(self._deploy)

    def _on_deploy_mode_changed(self, mode: str) -> None:
        if mode == self._deploy_mode:
            return
        self._deploy_mode = mode
        self._make_deploy_manager()
        mode_name = self.window.text(
            "mode_local" if mode == DEPLOY_LOCAL else "mode_remote"
        )
        self.window.flash(self.window.text("deploy_mode_switched", mode=mode_name))

    # ------------------------------------------------------------------
    # 请求处理
    # ------------------------------------------------------------------
    def _cfg(self) -> TeleopConfig:
        return self.window.read_config()

    def _on_start_gmr(self) -> None:
        if not self._validate_gmr():
            return
        self.gmr.start(self._cfg())

    def _on_stop_gmr(self) -> None:
        self.gmr.stop()

    def _on_reference_settings_changed(self) -> None:
        self.gmr.update_reference_settings(self._cfg())

    def _on_connect(self) -> None:
        if self._deploy is None:
            return
        if not self._validate_deploy():
            return
        if not self.gmr.update_reference_settings(self._cfg()):
            self.window.set_ssh_connected(False)
            return
        self._deploy.connect(self._cfg())
        # ] 键由 parse_deploy_line 在检测到 "Proceed with deployment" 后自动发送

    def _on_emergency(self) -> None:
        if self._deploy is not None:
            self._deploy.emergency_stop()

    def _on_toggle_tracking(self) -> None:
        if self._deploy is not None:
            self._deploy.send_enter()

    # ------------------------------------------------------------------
    # 校验
    # ------------------------------------------------------------------
    def _validate_gmr(self) -> bool:
        cfg = self._cfg()
        if not cfg.conda_python:
            self.window.warn(
                self.window.text("missing_conda_python"),
                self.window.text("enter_conda_python"),
            )
            return False
        if not cfg.gmr_root:
            self.window.warn(
                self.window.text("missing_gmr_root"),
                self.window.text("enter_gmr_root"),
            )
            return False
        return True

    def _validate_deploy(self) -> bool:
        cfg = self._cfg()
        if self._deploy_mode == DEPLOY_REMOTE:
            if not cfg.ssh_host:
                self.window.warn(
                    self.window.text("missing_ssh_host"),
                    self.window.text("enter_ssh_host"),
                )
                return False
        if not cfg.zmq_host:
            self.window.warn(
                self.window.text("missing_zmq_host"),
                self.window.text("select_zmq_host"),
            )
            return False
        if not cfg.deploy_dir:
            self.window.warn(
                self.window.text("missing_deploy_directory"),
                self.window.text("enter_deploy_directory"),
            )
            return False
        return True


def main() -> None:
    app = QApplication(sys.argv)
    ctrl = TeleopController()
    ctrl.window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
