from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "g1" / "modules" / "scripts"
SDK_HAND_PATH = SCRIPTS_DIR.parent / "sdk_hand.py"
CYCLE_PATH = SCRIPTS_DIR / "left_dex3_middle_finger_cycle.py"
SIGN_PATH = SCRIPTS_DIR / "dex3_piece_sign.py"


def _load_module(path: Path, name: str, monkeypatch: pytest.MonkeyPatch):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    return module


def load_cycle(monkeypatch: pytest.MonkeyPatch):
    sdk_hand = ModuleType("sdk_hand")
    sdk_hand.Dex3HandController = object
    sdk_hand.FINGER_TO_IDXS = {"middle": [3, 4]}

    def build_hand_msg(_targets, *, kp, kd, tau, timeout):
        del kp, kd, tau
        commands = [
            SimpleNamespace(mode=index + timeout * 100, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
            for index in range(7)
        ]
        return SimpleNamespace(motor_cmd=commands)

    sdk_hand.build_hand_msg = build_hand_msg
    sdk_hand.hand_grip_targets = lambda _side, percent: [float(percent)] * 7
    sdk_hand.pack_ris_mode = lambda motor_id, timeout=0: motor_id + timeout * 100
    monkeypatch.setitem(sys.modules, "sdk_hand", sdk_hand)
    return _load_module(CYCLE_PATH, "_test_left_dex3_cycle", monkeypatch)


def load_sign(monkeypatch: pytest.MonkeyPatch):
    sdk_wrapper = ModuleType("sdk_wrapper")
    sdk_wrapper.G1 = object
    sdk_wrapper.HAND_CLOSED = {
        "left": [10.0 + index for index in range(7)],
        "right": [20.0 + index for index in range(7)],
    }
    sdk_wrapper.HAND_OPEN = {
        "left": [30.0 + index for index in range(7)],
        "right": [40.0 + index for index in range(7)],
    }
    monkeypatch.setitem(sys.modules, "sdk_wrapper", sdk_wrapper)
    return _load_module(SIGN_PATH, "_test_dex3_piece_sign", monkeypatch)


def load_sdk_hand(monkeypatch: pytest.MonkeyPatch):
    package_names = (
        "unitree_sdk2py",
        "unitree_sdk2py.idl",
        "unitree_sdk2py.idl.unitree_hg",
        "unitree_sdk2py.idl.unitree_hg.msg",
        "unitree_sdk2py.core",
    )
    packages = {name: ModuleType(name) for name in package_names}
    for name, package in packages.items():
        package.__path__ = []
        monkeypatch.setitem(sys.modules, name, package)

    class HandCmd:
        pass

    class HandState:
        pass

    dds_messages = ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds_messages.HandCmd_ = HandCmd
    dds_messages.HandState_ = HandState
    monkeypatch.setitem(sys.modules, dds_messages.__name__, dds_messages)

    defaults = ModuleType("unitree_sdk2py.idl.default")
    defaults.unitree_hg_msg_dds__HandCmd_ = lambda: SimpleNamespace(
        motor_cmd=[SimpleNamespace() for _ in range(7)]
    )
    monkeypatch.setitem(sys.modules, defaults.__name__, defaults)

    channel = ModuleType("unitree_sdk2py.core.channel")

    class Publisher:
        def __init__(self, *_args):
            pass

        def Init(self):
            pass

        def Write(self, _message, timeout=None):
            return True

    class Subscriber:
        def __init__(self, *_args):
            pass

        def Init(self, *_args):
            pass

    channel.ChannelPublisher = Publisher
    channel.ChannelSubscriber = Subscriber
    packages["unitree_sdk2py.core"].channel = channel
    monkeypatch.setitem(sys.modules, channel.__name__, channel)

    dds_env = ModuleType("dds_env")
    dds_env.default_dds_iface = lambda _preferred: "eth0"
    dds_env.ensure_channel_factory_initialized = lambda *_args: None
    dds_env.ensure_cyclonedds_environment = lambda: None
    monkeypatch.setitem(sys.modules, "dds_env", dds_env)
    return _load_module(SDK_HAND_PATH, "_test_sdk_hand", monkeypatch)


@pytest.mark.parametrize(
    "arguments",
    [
        ["--domain-id", "233"],
        ["--open-s", "nan"],
        ["--hold-s", "-1"],
        ["--rate-hz", "201"],
        ["--kp", "inf"],
        ["--cycles", "-1"],
        ["--iface", "   "],
    ],
)
def test_cycle_cli_rejects_unsafe_values(
    monkeypatch: pytest.MonkeyPatch,
    arguments: list[str],
) -> None:
    cycle = load_cycle(monkeypatch)

    with pytest.raises(SystemExit):
        cycle.parse_args(arguments)


def test_cycle_cli_requires_distinct_ordered_endpoints(monkeypatch: pytest.MonkeyPatch) -> None:
    cycle = load_cycle(monkeypatch)

    args = cycle.parse_args(["--open-percent", "10", "--close-percent", "80", "--cycles", "2"])
    assert args.cycles == 2
    with pytest.raises(SystemExit):
        cycle.parse_args(["--open-percent", "80", "--close-percent", "80"])


def test_sdk_hand_rejects_non_finite_commands(monkeypatch: pytest.MonkeyPatch) -> None:
    sdk_hand = load_sdk_hand(monkeypatch)

    with pytest.raises(ValueError, match="percent"):
        sdk_hand.hand_grip_targets("left", float("nan"))
    with pytest.raises(ValueError, match="target"):
        sdk_hand.clamp_hand_targets("left", [0.0] * 6 + [float("inf")])
    with pytest.raises(ValueError, match="kp"):
        sdk_hand.build_hand_msg([0.0] * 7, kp=float("nan"), kd=0.1, tau=0.0)


def test_sdk_hand_validates_connection_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    sdk_hand = load_sdk_hand(monkeypatch)

    with pytest.raises(ValueError, match="domain_id"):
        sdk_hand.Dex3HandController("left", domain_id=233)
    with pytest.raises(ValueError, match="iface"):
        sdk_hand.Dex3HandController("left", iface="  ")


def test_sdk_hand_discards_non_finite_feedback(monkeypatch: pytest.MonkeyPatch) -> None:
    sdk_hand = load_sdk_hand(monkeypatch)
    controller = sdk_hand.Dex3HandController("left")
    bad_state = SimpleNamespace(
        motor_state=[SimpleNamespace(q=0.0) for _ in range(6)] + [SimpleNamespace(q=float("nan"))],
        press_sensor_state=[],
    )

    controller._state_cb(bad_state)

    assert controller.get_state_snapshot() is None


def test_middle_only_command_releases_other_joints(monkeypatch: pytest.MonkeyPatch) -> None:
    cycle = load_cycle(monkeypatch)

    class Controller:
        message = None

        def get_state_snapshot(self, max_age):
            assert max_age == pytest.approx(1.0)
            return {"positions": [0.0] * 7}

        def write_command_once(self, message):
            self.message = message
            return True

    controller = Controller()
    cycle.write_middle_only(
        controller,
        [-0.4, -0.7],
        kp=0.35,
        kd=0.08,
        max_state_age_s=1.0,
    )

    assert controller.message is not None
    for index, command in enumerate(controller.message.motor_cmd):
        if index in cycle.MIDDLE_IDXS:
            assert command.mode == index
            assert command.kp == pytest.approx(0.35)
            assert command.kd == pytest.approx(0.08)
        else:
            assert command.mode == index + 100
            assert command.kp == 0.0
            assert command.kd == 0.0


def test_middle_only_stops_when_feedback_is_stale(monkeypatch: pytest.MonkeyPatch) -> None:
    cycle = load_cycle(monkeypatch)

    class Controller:
        def get_state_snapshot(self, max_age):
            return None

        def write_command_once(self, message):
            raise AssertionError("stale feedback must prevent command publication")

    with pytest.raises(RuntimeError, match="feedback"):
        cycle.write_middle_only(
            Controller(),
            [0.0, 0.0],
            kp=0.35,
            kd=0.08,
            max_state_age_s=1.0,
        )


@pytest.mark.parametrize(
    "arguments",
    [
        ["--domain-id", "-1"],
        ["--close-hold-s", "nan"],
        ["--open-hold-s", "61"],
        ["--rate-hz", "0"],
        ["--ramp-s", "inf"],
        ["--iface", ""],
    ],
)
def test_piece_sign_cli_rejects_unsafe_values(
    monkeypatch: pytest.MonkeyPatch,
    arguments: list[str],
) -> None:
    sign = load_sign(monkeypatch)

    with pytest.raises(SystemExit):
        sign.parse_args(arguments)


def test_piece_sign_targets_keep_thumb_closed_and_open_fingers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sign = load_sign(monkeypatch)

    assert sign.piece_sign_targets("left") == [10.0, 11.0, 12.0, 33.0, 34.0, 35.0, 36.0]
    with pytest.raises(ValueError, match="one hand"):
        sign.piece_sign_targets("both")


def _sign_args() -> argparse.Namespace:
    return argparse.Namespace(
        iface="eth0",
        domain_id=0,
        hand="left",
        close_hold_s=0.8,
        open_hold_s=1.5,
        rate_hz=50.0,
        ramp_s=0.5,
    )


def test_piece_sign_closes_before_opening_fingers(monkeypatch: pytest.MonkeyPatch) -> None:
    sign = load_sign(monkeypatch)
    events = []

    class Robot:
        def __init__(self, **kwargs):
            events.append(("connect", kwargs))

        def close_dex3_hand(self, **kwargs):
            events.append(("close", kwargs))
            return {"hand": "left", "ok": True}

        def hand_pose(self, targets, **kwargs):
            events.append(("pose", {"targets": targets, **kwargs}))
            return {"hand": "left", "ok": True}

    monkeypatch.setattr(sign, "G1", Robot)
    monkeypatch.setattr(sign, "parse_args", lambda: _sign_args())

    assert sign.main() == 0
    assert [event[0] for event in events] == ["connect", "close", "pose"]


def test_piece_sign_aborts_if_close_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    sign = load_sign(monkeypatch)

    class Robot:
        def __init__(self, **_kwargs):
            pass

        def close_dex3_hand(self, **_kwargs):
            return {"hand": "left", "ok": False, "error": "offline"}

        def hand_pose(self, *_args, **_kwargs):
            raise AssertionError("piece sign must not run after a failed close")

    monkeypatch.setattr(sign, "G1", Robot)
    monkeypatch.setattr(sign, "parse_args", lambda: _sign_args())

    assert sign.main() == 1
