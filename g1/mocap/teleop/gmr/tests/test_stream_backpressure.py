import io
import json

import numpy as np
import zmq

from general_motion_retargeting.utils.moretion_stream import (
    MoretionStreamReceiver,
    MoretionStreamToGMR,
)
from scripts.moretion_stream_to_sonic import (
    _GmrFrameBuffer,
    _build_parser,
    _build_pose_batch,
    _build_live_pose_batch,
    _build_preview_pose_batch,
    _reference_timing,
    _ReferenceSettingsUpdates,
    SonicZmqPublisher,
)
from teleop_studio.process_manager import parse_deploy_line


class _EmptyStreamSocket:
    def recv_multipart(self, flags=0):
        raise zmq.ZMQError(zmq.EAGAIN)


def test_tcp_receiver_keeps_latest_frame_from_one_chunk():
    receiver = MoretionStreamReceiver("localhost", 9999)
    frames = [
        {"frame": 1, "device": "test"},
        {"frame": 2, "device": "test"},
        {"frame": 3, "device": "test"},
    ]
    receiver._tcp_buffer = b"\n".join(
        json.dumps(frame).encode("utf-8") for frame in frames
    )
    receiver._zmq_sock = _EmptyStreamSocket()

    assert receiver._recv_tcp() == frames[-1]
    assert receiver._tcp_buffer == b""
    assert receiver.dropped_count == 2


def test_tcp_parser_preserves_non_ascii_frame_boundaries():
    receiver = MoretionStreamReceiver("localhost", 9999)
    first = {"frame": 1, "device": "\u6d4b\u8bd5"}
    second = {"frame": 2}
    receiver._tcp_buffer = (
        json.dumps(first, ensure_ascii=False).encode("utf-8")
        + json.dumps(second).encode("utf-8")
    )

    assert receiver._try_parse_buffer() == first
    assert receiver._try_parse_buffer() == second
    assert receiver._tcp_buffer == b""


def test_frame_windows_advance_once_per_tick_after_warmup():
    buffer = _GmrFrameBuffer(size=3)
    windows = []

    for frame_index in range(6):
        buffer.push({"frame_index": frame_index})
        window = buffer.get_full_window()
        if window is not None:
            windows.append([frame["frame_index"] for frame in window])

    assert windows == [[0, 1, 2], [1, 2, 3], [2, 3, 4], [3, 4, 5]]
    assert len(buffer.frames) == 3


def test_live_settings_coalesce_and_preserve_preview_samples():
    updates = _ReferenceSettingsUpdates(0.02)
    updates.read_stream(io.StringIO(
        '{"model_profile":"default","preview_delay_ms":400}\n'
        '{"model_profile":"low_latency","preview_delay_ms":100}\n'
        '{"model_profile":"unknown","preview_delay_ms":100}\n'
        '{"model_profile":"default","preview_delay_ms":-1}\n'
        'invalid json\n'
    ))
    assert updates.take_latest() == ("low_latency", 100, 10, 5)
    assert updates.take_latest() is None

    buffer = _GmrFrameBuffer(25)
    for frame_index in range(25):
        buffer.push({"frame_index": frame_index})
    buffer.resize(10)
    assert [frame["frame_index"] for frame in buffer.frames] == list(range(15, 25))
    updates.read_stream(io.StringIO('{"model_profile":"default","preview_delay_ms":null}\n'))
    profile, delay_ms, horizon, ticks = updates.take_latest()
    assert (profile, delay_ms, horizon, ticks) == ("default", None, 46, 46)
    buffer.resize(5 + ticks)
    assert buffer.get_full_window() is None
    for frame_index in range(25, 66):
        buffer.push({"frame_index": frame_index})
    assert [frame["frame_index"] for frame in buffer.get_full_window()] == list(range(15, 66))


def test_default_batch_keeps_stable_reference_window():
    args = _build_parser().parse_args([])
    assert args.batch == 2
    assert args.reference_mode == "preview"
    assert args.model_profile == "default"
    assert args.preview_delay_ms is None
    assert args.live_future_frames >= 46


def _pose_frame(index, position):
    return {
        "frame_index": index,
        "joint_pos": np.full(29, position, dtype=float),
        "body_quat": np.array([1.0, 0.0, 0.0, 0.0]),
        "root_pos": np.array([position, 0.0, 1.0]),
    }


def test_preview_defaults_preserve_each_encoder_and_final_forward_velocity():
    for profile, horizon, step in (("default", 46, 5), ("low_latency", 10, 1)):
        assert _reference_timing(profile, 0.02, None) == (horizon, horizon)
        frames = [_pose_frame(i, (i * 0.02) ** 2) for i in range(horizon + 9)]
        batch, current = _build_preview_pose_batch(frames, 0.02, horizon, horizon)
        jp, jv, _, rp, indices = batch
        assert current == frames[-1]["frame_index"] - horizon
        current_offset = int(np.flatnonzero(indices == current)[0])
        sample_offsets = current_offset + np.arange(10) * step
        source_indices = current + np.arange(10) * step
        np.testing.assert_allclose(jp[sample_offsets, 0], (source_indices * 0.02) ** 2)
        np.testing.assert_allclose(rp[sample_offsets, 0], jp[sample_offsets, 0])
        expected_vel = (((source_indices + 1) * 0.02) ** 2 - (source_indices * 0.02) ** 2) / 0.02
        np.testing.assert_allclose(jv[sample_offsets, 0], expected_vel)


def test_preview_waits_once_and_never_rewinds_after_warmup():
    history = []
    cursors = []
    for tick in range(60):
        history.append(_pose_frame(tick, tick * 0.02))
        result = _build_preview_pose_batch(history[-48:], 0.02, 46, 46)
        if tick < 46:
            assert result is None
        else:
            cursors.append(result[1])
    assert cursors == list(range(14))


def test_short_preview_keeps_available_future_before_holding_missing_samples():
    frames = [_pose_frame(i, i * 0.02) for i in range(25)]
    (jp, jv, _, _, indices), current = _build_preview_pose_batch(frames, 0.02, 46, 20)
    assert current == 4
    assert indices[0] == current - 1
    np.testing.assert_allclose(jp[:22, 0], np.arange(3, 25) * 0.02)
    np.testing.assert_allclose(jp[22:], 0.48)
    np.testing.assert_allclose(jv[22:], 0.0)
    assert indices[-1] == current + 45


def test_zero_preview_reproduces_live_hold_without_waiting():
    frames = [_pose_frame(i, i * 0.02) for i in range(5)]
    batch, current = _build_preview_pose_batch(frames, 0.02, 50, 0)
    assert current == 4
    expected = _build_live_pose_batch(frames[-2:], 0.02)
    for actual, reference in zip(batch, expected):
        np.testing.assert_allclose(actual, reference)


def test_preview_custom_delay_rounds_up_and_rejects_invalid_values():
    assert _reference_timing("default", 0.02, 401) == (46, 21)
    for delay in (-1.0, float("nan"), float("inf")):
        try:
            _reference_timing("default", 0.02, delay)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid preview accepted")


def test_reduced_history_keeps_identical_encoder_inputs_and_preview_delay():
    # Nonlinear joint/root motion exercises the forward velocity corrections.
    frames = [_pose_frame(i, (i * 0.02) ** 2) for i in range(100)]
    for profile, step, short_delay in (("default", 5, 400), ("low_latency", 1, 100)):
        for delay_ms in (0, short_delay, None):
            horizon, ticks = _reference_timing(profile, 0.02, delay_ms)
            old_batch, old_current = _build_preview_pose_batch(
                frames[-(5 + ticks):], 0.02, horizon, ticks, history_frames=5,
            )
            new_batch, current = _build_preview_pose_batch(
                frames[-(2 + ticks):], 0.02, horizon, ticks,
            )
            assert current == old_current == 99 - ticks
            assert new_batch[-1][0] == current - 1
            assert len(new_batch[-1]) == len(old_batch[-1]) - 3
            # Everything at/after the target stays identical, including the
            # final derivative frame when full measured preview is selected.
            for old, new in zip(old_batch, new_batch):
                np.testing.assert_array_equal(new[1:], old[4:])
            sample_offsets = 1 + np.arange(10) * step
            assert sample_offsets[-1] < len(new_batch[-1])
            np.testing.assert_array_equal(new_batch[-1][sample_offsets], current + np.arange(10) * step)


def test_live_reference_sends_first_sample_without_history_warmup():
    jp, jv, bq, rp, indices = _build_live_pose_batch([_pose_frame(0, 0.2)], 0.02)

    assert jp.shape == (50, 29)
    np.testing.assert_allclose(jp, 0.2)
    np.testing.assert_allclose(jv, 0.0)
    np.testing.assert_array_equal(indices, np.arange(50))
    np.testing.assert_allclose(bq, np.tile([1.0, 0.0, 0.0, 0.0], (50, 1)))
    np.testing.assert_allclose(rp, np.tile([0.2, 0.0, 1.0], (50, 1)))


def test_live_reference_retains_history_and_holds_unknown_future():
    frames = [_pose_frame(i, i * 0.02) for i in range(5)]
    jp, jv, _, rp, indices = _build_live_pose_batch(frames, 0.02)

    assert len(indices) == 54
    np.testing.assert_allclose(jp[:5, 0], np.arange(5) * 0.02)
    np.testing.assert_allclose(jv[:5], 1.0)
    np.testing.assert_allclose(jp[5:], 0.08)
    np.testing.assert_allclose(jv[5:], 0.0)
    np.testing.assert_allclose(rp[5:, 0], 0.08)
    # Both encoder lookahead strides fit at the live cursor, index 4.
    for step in (1, 5):
        np.testing.assert_allclose(jp[4 + np.arange(10) * step], 0.08)
    np.testing.assert_array_equal(indices, np.arange(54))


def test_velocity_uses_elapsed_ticks_when_output_skips_frames():
    frames = [_pose_frame(10, 0.0), _pose_frame(15, 0.1)]
    _, jv, _, _, _ = _build_live_pose_batch(frames, 0.02)

    np.testing.assert_allclose(jv[:2], 1.0)
    np.testing.assert_allclose(jv[2:], 0.0)


def test_live_current_sample_is_serialized_separately_from_future_tail():
    class Socket:
        def send(self, message):
            self.message = message

    publisher = object.__new__(SonicZmqPublisher)
    publisher._socket = Socket()
    jp, jv, bq, rp, indices = _build_live_pose_batch([_pose_frame(12, 0.1)], 0.02)
    publisher.send_pose(jp, jv, bq, indices, root_pos=rp, live_frame_index=12)
    packet = publisher._socket.message
    header = json.loads(packet[4:1284].rstrip(b"\0"))
    cursor_field = header["fields"][-1]

    assert cursor_field == {"name": "live_frame_index", "dtype": "i64", "shape": [1]}
    assert int(np.frombuffer(packet[-8:], dtype="<i8")[0]) == 12
    assert indices[-1] == 61


def test_rolling_window_corrects_previous_tail_velocity():
    def frame(index, joint_pos):
        return {
            "frame_index": index,
            "joint_pos": np.full(29, joint_pos, dtype=float),
            "body_quat": np.array([1.0, 0.0, 0.0, 0.0]),
            "root_pos": np.zeros(3),
        }

    first_window = [frame(0, 0.0), frame(1, 1.0), frame(2, 3.0)]
    second_window = [frame(1, 1.0), frame(2, 3.0), frame(3, 6.0)]

    _, first_vel, _, _, _ = _build_pose_batch(first_window, output_period=1.0)
    _, second_vel, _, _, indices = _build_pose_batch(
        second_window, output_period=1.0
    )

    # Frame 2 initially copies the last known slope, then receives its true
    # forward difference when it becomes an interior frame in the next window.
    np.testing.assert_allclose(first_vel[-1], 2.0)
    np.testing.assert_allclose(second_vel[1], 3.0)
    np.testing.assert_array_equal(indices, [1, 2, 3])


def test_deploy_log_parser_tracks_all_zmq_toggle_states():
    tracking_states = []
    control_states = []
    answers = []

    def parse(line):
        parse_deploy_line(
            line,
            control_states.append,
            tracking_states.append,
            answers.append,
        )

    parse("ZMQ STREAMING MODE: ENABLED")
    parse("ZMQ STREAMING MODE: DISABLED")
    parse("ZMQ STREAMING MODE: FORCE DISABLED")

    assert tracking_states == [True, False, False]
    assert control_states == []
    assert answers == []


def test_root_compensation_uses_one_fixed_heading_for_pose_and_translation():
    converter = MoretionStreamToGMR(verbose=False)

    yaw = np.deg2rad(90.0)
    root_quat = np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])
    first_quats = np.zeros((1, 1, 4), dtype=float)
    first_quats[0, 0] = root_quat
    first_positions = np.array([[[2.0, 3.0, 1.0]]])
    converter._apply_root_compensation(first_quats, first_positions)

    np.testing.assert_allclose(first_quats[0, 0], [1.0, 0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(first_positions[0, 0], [0.0, 0.0, 1.0], atol=1e-12)

    second_quats = np.zeros((1, 1, 4), dtype=float)
    second_quats[0, 0] = root_quat
    second_positions = np.array([[[3.0, 3.0, 1.0]]])
    converter._apply_root_compensation(second_quats, second_positions)

    # A +X displacement becomes -Y after removing the fixed +90 degree heading.
    np.testing.assert_allclose(second_quats[0, 0], [1.0, 0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(second_positions[0, 0], [0.0, -1.0, 1.0], atol=1e-12)


def test_converter_selects_requested_device():
    converter = MoretionStreamToGMR(verbose=False, device_id="actor-2")
    frame = converter.convert_frame(
        {
            "deviceList": [
                {"deviceId": "actor-1", "coordinate": [1.0, 0.0, 0.0]},
                {"deviceId": "actor-2", "coordinate": [2.0, 0.0, 0.0]},
            ]
        }
    )

    # The initial horizontal position is zeroed, but the selected actor is
    # retained as the converter's fixed position reference.
    np.testing.assert_allclose(converter._init_pos_xz, [0.0, 2.0])
    np.testing.assert_allclose(frame["Hips"][0][:2], [0.0, 0.0])
