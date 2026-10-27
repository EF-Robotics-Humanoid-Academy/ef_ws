import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import teleop_studio.main_window as main_window
from teleop_studio.main_window import MainWindow


def test_language_switch_retranslates_ui_without_losing_state(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setattr(main_window, "list_network_interfaces", lambda: {})
    app = QApplication.instance() or QApplication([])
    window = MainWindow()
    window.set_language("zh_CN")
    window._moretion_ip_edit.setText("10.0.0.8")
    window.set_gmr_running(True)
    window.set_ssh_connected(True)
    window.set_control_state("control")
    window.set_tracking_state(True)

    window.set_language("en")

    assert window.language() == "en"
    assert window.windowTitle().endswith("Teleoperation")
    assert window._param_group.title() == "Runtime Parameters"
    assert window._param_labels["network_interface"].text() == "Local Network Interface:"
    assert window._deploy_mode_combo.itemText(0).startswith("Remote SSH")
    assert window._deploy_target_combo.itemText(0) == "MuJoCo Simulation"
    assert window._model_profile_combo.itemText(1) == "Low-latency Model"
    assert window.led_gmr._state.text() == "Running"
    assert window.led_control._state.text() == "In control"
    assert window.led_tracking._state.text() == "Tracking"
    assert window.btn_robot.text() == "E-Stop"
    assert window.btn_toggle_tracking.text() == "Pause\nTracking"
    assert window._moretion_ip_edit.text() == "10.0.0.8"
    assert window.read_config().zmq_host == ""

    local_index = window._deploy_mode_combo.findData("local")
    if local_index >= 0:
        window._deploy_mode_combo.setCurrentIndex(local_index)
        assert window.read_config().zmq_host == "localhost"

    window.close()
    app.processEvents()


def test_selected_language_is_persisted(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    monkeypatch.setattr(main_window, "list_network_interfaces", lambda: {})
    app = QApplication.instance() or QApplication([])
    first_window = MainWindow()
    first_window.set_language("en")
    first_window._settings.sync()
    first_window.close()

    second_window = MainWindow()

    assert second_window.language() == "en"
    assert second_window._language_menu.title() == "Language"
    second_window.close()
    app.processEvents()
