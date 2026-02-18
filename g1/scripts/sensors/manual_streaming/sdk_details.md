# RGBD Access in `sensors/manual_streaming`

This document explains how to access RGBD data in this folder using:

1. Intel RealSense SDK (`pyrealsense2`) for direct camera access.
2. Manual network streaming (GStreamer/UDP and ZeroMQ) for remote consumption.

Scope: files in `scripts/sensors/manual_streaming`.

---

## 1. Quick Architecture

There are two practical pipelines in this directory:

1. Direct SDK pipeline (local camera access):
   - Capture directly from RealSense with `pyrealsense2`.
   - Best for local processing, calibration work, and direct depth math.
   - Scripts: `stream_realsense.py`, `stream_realsense_clip_can.py`, `image_server.py`, `list_realsense_devices.py`.

2. Manual network streaming pipeline (remote receiver):
   - Sender captures frames via RealSense SDK, compresses with GStreamer, sends over UDP.
   - Receiver decodes with GStreamer `appsink` and uses NumPy/OpenCV.
   - Best for low-latency offboard viewing/inference.
   - Sender: `jetson_realsense_stream.py`
   - Receivers: `receive_realsense_gst.py`, `receive_realsense_gst_clip_can.py`, `real_time_detection.py`.

There is also a ZeroMQ image server path in `image_server.py` that publishes JPEG+PNG multipart messages.

---

## 2. What “RGBD” Means Here

In this folder, RGBD is represented as:

- RGB/color frame:
  - Format: BGR8 (`rs.format.bgr8`)
  - Typical shape: `(H, W, 3)`, `uint8`

- Depth frame:
  - Raw depth format at capture: Z16 (`rs.format.z16`)
  - Typical shape: `(H, W)`, `uint16`
  - Units:
    - Raw depth values are in camera depth units; convert with `depth_scale` for meters.
    - `image_server.py` reads `depth_scale` via `first_depth_sensor().get_depth_scale()`.

Depth is typically aligned to color (`rs.align(rs.stream.color)`), so pixel `(u, v)` in color corresponds to pixel `(u, v)` in depth.

---

## 3. Direct SDK Access (Local, Recommended Baseline)

### 3.1 Device discovery

Use:

```bash
python3 list_realsense_devices.py
```

This prints model, serial, firmware, and USB port when available.

### 3.2 Minimal direct viewer

Use:

```bash
python3 stream_realsense.py --width 640 --height 480 --fps 30
```

Optional flags:

- `--infra`: enable and display left/right IR streams.
- `--imu`: print gyro/accel (if camera supports IMU, e.g., D435i).

Key internals in `stream_realsense.py`:

- Enables streams:
  - `rs.stream.depth` as `z16`
  - `rs.stream.color` as `bgr8`
- Aligns depth to color with `rs.align(rs.stream.color)`.
- Applies filters:
  - `rs.spatial_filter()`
  - `rs.temporal_filter()`
- Displays side-by-side RGB + colorized depth.

### 3.3 Accessing intrinsics and depth scale

- Intrinsics:
  - `profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()`
- Depth scale:
  - `profile.get_device().first_depth_sensor().get_depth_scale()`

Use this for metric depth conversion:

```python
depth_meters = depth_image_uint16[v, u] * depth_scale
```

### 3.4 Multiple RealSense devices

Lock to a specific camera serial:

```python
config.enable_device("<serial>")
```

This is already supported in `image_server.py` (`RealSenseCamera(serial_number=...)`).

---

## 4. Manual Streaming via GStreamer/UDP

## 4.1 Sender (`jetson_realsense_stream.py`)

Run on robot/Jetson side:

```bash
python3 jetson_realsense_stream.py --client-ip <LAPTOP_IP> --width 640 --height 480 --fps 30
```

Transport details:

- RGB stream:
  - UDP port: `5600`
  - RTP payload type: `96`
  - Codec: H.264
- Depth stream:
  - UDP port: `5602`
  - RTP payload type: `97`
  - Codec: H.264
  - Source depth is Z16, but sender colorizes depth to BGR (`COLORMAP_PLASMA`) before encoding.

Important implementation note:

- The current code sends depth as colorized BGR video, not raw 16-bit depth.
- `receive_realsense_gst.py` and `receive_realsense_gst_clip_can.py` treat incoming depth as BGR visualization.

### 4.2 Receiver (`receive_realsense_gst.py`)

Run on laptop/workstation:

```bash
python3 receive_realsense_gst.py
```

What it does:

- Builds two GStreamer pipelines with `udpsrc -> rtph264depay -> avdec_h264 -> videoconvert -> appsink`.
- Pulls samples from appsink (`try-pull-sample`).
- Converts payload to NumPy:
  - RGB: `(H, W, 3) uint8`
  - Depth visualization: `(H, W, 3) uint8`
- Displays side-by-side.

### 4.3 Receiver with CLIP + navigation trigger (`receive_realsense_gst_clip_can.py`)

Example:

```bash
python3 receive_realsense_gst_clip_can.py \
  --rgb-port 5600 --depth-port 5602 --width 640 --height 480 --fps 30 \
  --threshold 0.85 --every 15 --downscale 0.5
```

Optional nav trigger:

```bash
python3 receive_realsense_gst_clip_can.py \
  --trigger-nav-on-detect \
  --nav-map <path_to_map.npz>
```

Helper wrapper:

```bash
./run_clip_can_nav.sh [optional_map_path]
```

---

## 5. ZeroMQ Manual Streaming (`image_server.py`)

This is an alternative manual stream path:

- Captures with RealSense SDK.
- Publishes multipart ZeroMQ `PUB` messages on TCP port `5555` by default.

Message structure for RealSense mode:

1. `color_jpg` (JPEG-encoded color frame)
2. `depth_png` (PNG-encoded raw depth frame)
3. `depth_scale_bytes` (packed float32, `struct.pack('f', scale)`)

Key properties:

- Depth remains raw 16-bit when encoded in PNG.
- Better if remote side needs metric depth reconstruction.
- Uses `rs.align(rs.stream.color)` before publish.

---

## 6. Dependencies

Common:

- `pyrealsense2`
- `numpy`
- `opencv-python` (or system OpenCV with HighGUI)

For GStreamer receivers/sender:

```bash
sudo apt install -y \
  python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
```

Python packages:

```bash
python3 -m pip install --upgrade numpy opencv-python pyrealsense2
```

For CLIP scripts:

- `torch`
- `open_clip_torch`
- `Pillow`

---

## 7. Network and Runtime Requirements

- Sender and receiver must be on reachable IP network (no blocked UDP 5600/5602).
- Use consistent resolution/FPS on both sides.
- If stream does not appear:
  - verify receiver IP passed to sender `--client-ip`
  - verify ports are open
  - verify camera is detected with `list_realsense_devices.py`

---

## 8. Data Semantics and Conversion Notes

1. Depth units:
   - Raw depth is not meters unless multiplied by `depth_scale`.

2. Alignment:
   - If depth is aligned to color, `(u,v)` corresponds across modalities.
   - This is required before sampling depth at detected RGB bounding boxes.

3. Visualized vs raw depth:
   - `jetson_realsense_stream.py` currently sends colorized depth image.
   - For quantitative depth at receiver, use:
     - local SDK direct capture, or
     - `image_server.py` path with depth PNG + `depth_scale`.

---

## 9. Canonical Workflows

## Workflow A: Local RGBD access for metric perception

1. `python3 list_realsense_devices.py`
2. `python3 stream_realsense.py --width 640 --height 480 --fps 30`
3. Integrate depth math in your app:
   - get aligned depth frame
   - get depth scale
   - convert `depth_raw * depth_scale`

## Workflow B: Robot-to-laptop visual stream (manual UDP)

1. On Jetson:
   - `python3 jetson_realsense_stream.py --client-ip <laptop_ip> --width 640 --height 480 --fps 30`
2. On laptop:
   - `python3 receive_realsense_gst.py`
3. For CLIP detection:
   - use `receive_realsense_gst_clip_can.py` or `real_time_detection.py`

## Workflow C: Remote raw-depth transport

1. Run `image_server.py` (RealSense mode).
2. Subscribe via ZeroMQ and decode multipart:
   - JPEG color
   - PNG depth (uint16)
   - float depth scale

---

## 10. Known Caveats in Current Scripts

1. `jetson_realsense_stream.py` header comments mention raw/RFC4175 depth format, but current implementation sends H.264 colorized depth BGR.
2. `receive_realsense_gst.py` includes a `colourise_depth` helper that is not used in current flow because depth is already colorized upstream.
3. `image_server.py` has minor legacy code in unit-test message assembly, but normal non-test path for multipart publish is clear and functional.

---

## 11. Minimal SDK Snippet (Aligned RGB + Raw Depth + Scale)

```python
import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

frames = pipeline.wait_for_frames()
aligned = align.process(frames)
color = np.asanyarray(aligned.get_color_frame().get_data())      # (480,640,3) uint8
depth = np.asanyarray(aligned.get_depth_frame().get_data())      # (480,640) uint16

u, v = 320, 240
z_m = float(depth[v, u]) * depth_scale
print("depth[m] at center:", z_m)

pipeline.stop()
```

---

## 12. File Map

- `list_realsense_devices.py`: enumerate connected devices.
- `stream_realsense.py`: direct SDK RGBD viewer with alignment and filters.
- `stream_realsense_clip_can.py`: direct SDK RGB + CLIP detection.
- `jetson_realsense_stream.py`: sender (RealSense SDK -> GStreamer RTP/UDP).
- `receive_realsense_gst.py`: receiver (GStreamer RTP/UDP -> NumPy/OpenCV).
- `receive_realsense_gst_clip_can.py`: receiver + CLIP + optional nav trigger.
- `real_time_detection.py`: RGB-only GStreamer receiver + periodic CLIP captions.
- `image_server.py`: ZeroMQ publisher for JPEG color + PNG depth + depth scale.
