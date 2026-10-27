import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import teleop_studio.main_window as main_window
from teleop_studio.main_window import MainWindow, TeleopConfig
from teleop_studio.app import TeleopController


def test_default_model_uses_deploy_defaults():
    command = TeleopConfig(zmq_host="localhost").deploy_command()

    assert " --cp " not in command
    assert " --obs-config " not in command
    assert "--input-type zmq" in command
    assert "--zmq-host localhost --zmq-port 5556 --zmq-topic pose" in command
    assert "--zmq-conflate" in command
    assert command.endswith(" sim")


def test_low_latency_model_adds_checkpoint_and_observation_config():
    command = TeleopConfig(
        zmq_host="localhost",
        model_profile="low_latency",
    ).deploy_command()

    assert "--cp policy/low_latency/model" in command
    assert "--obs-config policy/low_latency/observation_config.yaml" in command
    assert command.index("--cp") < command.index("--input-type")


def test_retargeting_preview_matches_model_and_custom_delay():
    for profile in ("default", "low_latency"):
        cfg = TeleopConfig(model_profile=profile)
        argv = cfg.moretion_command()
        assert argv[argv.index("--reference_mode") + 1] == "preview"
        assert argv[argv.index("--model_profile") + 1] == profile
        assert "--preview_delay_ms" not in argv
        cfg.preview_delay_ms = 120
        argv = cfg.moretion_command()
        assert argv[argv.index("--preview_delay_ms") + 1] == "120"


def test_zmq_conflate_can_be_disabled_for_buffered_playback():
    command = TeleopConfig(
        zmq_host="localhost",
        zmq_conflate=False,
    ).deploy_command()

    assert "--zmq-conflate" not in command


def test_model_selection_is_read_from_gui(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(main_window, "list_network_interfaces", lambda: {})
    window = MainWindow()

    assert window.read_config().model_profile == "default"
    assert window.read_config().preview_delay_ms == 400
    argv = window.read_config().moretion_command()
    assert argv[argv.index("--preview_delay_ms") + 1] == "400"

    low_latency_index = window._model_profile_combo.findData("low_latency")
    window._model_profile_combo.setCurrentIndex(low_latency_index)

    assert window.read_config().model_profile == "low_latency"
    assert window.read_config().preview_delay_ms == 100
    argv = window.read_config().moretion_command()
    assert argv[argv.index("--preview_delay_ms") + 1] == "100"
    window._preview_delay_spin.setValue(-1)
    assert "--preview_delay_ms" not in window.read_config().moretion_command()
    window._model_profile_combo.setCurrentIndex(window._model_profile_combo.findData("default"))
    assert window.read_config().preview_delay_ms == 400
    window.set_gmr_running(True)
    assert window._model_profile_combo.isEnabled()
    assert window._preview_delay_spin.isEnabled()
    updates = []
    window.reference_settings_changed.connect(lambda: updates.append(window.read_config()))
    window._model_profile_combo.setCurrentIndex(low_latency_index)
    assert len(updates) == 1
    assert updates[-1].model_profile == "low_latency"
    assert updates[-1].preview_delay_ms == 100
    window._preview_delay_spin.setValue(120)
    assert updates[-1].preview_delay_ms == 120
    window._robot_transition = True
    window._sync_buttons()
    assert not window._model_profile_combo.isEnabled()
    assert not window._preview_delay_spin.isEnabled()
    window._robot_transition = False
    window.set_ssh_connected(True)
    assert not window._model_profile_combo.isEnabled()
    assert not window._preview_delay_spin.isEnabled()
    window.set_ssh_connected(False)
    assert window._model_profile_combo.isEnabled()
    assert window._preview_delay_spin.isEnabled()
    window.set_gmr_running(False)
    assert window._model_profile_combo.isEnabled()
    window.close()
    app.processEvents()


def test_running_retargeter_receives_settings_and_deploy_sync_failure_blocks_connect(monkeypatch):
    import io
    import json
    from types import SimpleNamespace

    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(main_window, "list_network_interfaces", lambda: {})
    controller = TeleopController()
    pipe = io.BytesIO()
    controller.gmr._proc = SimpleNamespace(poll=lambda: None, stdin=pipe)
    window = controller.window
    window.set_gmr_running(True)
    window._model_profile_combo.setCurrentIndex(window._model_profile_combo.findData("low_latency"))
    window._preview_delay_spin.setValue(-1)
    settings = [json.loads(line) for line in pipe.getvalue().splitlines()]
    assert settings == [
        {"model_profile": "low_latency", "preview_delay_ms": 100},
        {"model_profile": "low_latency", "preview_delay_ms": None},
    ]
    connections = []
    controller._deploy = SimpleNamespace(connect=lambda cfg: connections.append(cfg))
    monkeypatch.setattr(controller, "_validate_deploy", lambda: True)
    controller._on_connect()
    assert len(connections) == 1
    assert connections[0].model_profile == "low_latency"
    pipe.close()
    window._robot_transition = True
    controller._on_connect()
    assert len(connections) == 1
    assert not window._robot_transition
    controller.gmr._proc = None
    window.close()
    app.processEvents()


def test_deploy_target_is_independent_from_local_or_remote_mode(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(main_window, "list_network_interfaces", lambda: {})
    window = MainWindow()

    assert window.read_config().deploy_target == "sim"

    real_index = window._deploy_target_combo.findData("real")
    window._deploy_target_combo.setCurrentIndex(real_index)
    assert window.read_config().deploy_target == "real"

    local_index = window._deploy_mode_combo.findData("local")
    window._deploy_mode_combo.setCurrentIndex(local_index)
    assert window.read_config().deploy_target == "real"
    assert window._deploy_target_combo.isEnabled()

    window.close()
    app.processEvents()
