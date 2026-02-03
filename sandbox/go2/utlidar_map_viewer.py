import argparse
import struct
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LidarState_


TOPIC_MAP_STATE = "rt/utlidar/map_state"
TOPIC_DESKEW = "rt/utlidar/cloud_deskewed"

POINTFIELD_FLOAT32 = 7


def _get_point_offsets(fields):
    offsets = {}
    for f in fields:
        offsets[f.name] = (f.offset, f.datatype)
    return offsets


def _extract_points_xy(msg: PointCloud2_, max_points: int = 2000):
    offsets = _get_point_offsets(msg.fields)
    if "x" not in offsets or "y" not in offsets:
        return []
    x_off, x_type = offsets["x"]
    y_off, y_type = offsets["y"]
    if x_type != POINTFIELD_FLOAT32 or y_type != POINTFIELD_FLOAT32:
        return []

    total = int(msg.width) * int(msg.height)
    if total <= 0:
        return []
    step = max(1, total // max_points)
    data = bytes(msg.data)
    endian = ">" if msg.is_bigendian else "<"
    pts = []
    for i in range(0, total, step):
        base = i * msg.point_step
        try:
            x = struct.unpack_from(endian + "f", data, base + x_off)[0]
            y = struct.unpack_from(endian + "f", data, base + y_off)[0]
        except struct.error:
            break
        pts.append((x, y))
    return pts


class MapViewer:
    def __init__(self, map_state_type: str):
        self.map_state_type = map_state_type
        self.map_state = None
        self.cloud_msg = None
        self._last_cloud_time = 0.0
        self._last_map_time = 0.0

    def map_state_cb(self, msg):
        self.map_state = msg
        self._last_map_time = time.time()

    def cloud_cb(self, msg: PointCloud2_):
        self.cloud_msg = msg
        self._last_cloud_time = time.time()

    def print_status(self):
        now = time.time()
        if self.map_state is not None and now - self._last_map_time < 1.0:
            if isinstance(self.map_state, HeightMap_):
                w = int(self.map_state.width)
                h = int(self.map_state.height)
                data = list(self.map_state.data)
                if data:
                    print(
                        f"[map_state] HeightMap {w}x{h} res={self.map_state.resolution:.3f} "
                        f"min={min(data):.3f} max={max(data):.3f}"
                    )
                else:
                    print(f"[map_state] HeightMap {w}x{h} res={self.map_state.resolution:.3f} (empty)")
            elif isinstance(self.map_state, String_):
                print(f"[map_state] {self.map_state.data}")
            elif isinstance(self.map_state, LidarState_):
                print(
                    f"[map_state] LidarState cloud_size={self.map_state.cloud_size} "
                    f"cloud_loss={self.map_state.cloud_packet_loss_rate:.3f}"
                )
            else:
                print("[map_state] updated")

        if self.cloud_msg is not None and now - self._last_cloud_time < 1.0:
            total = int(self.cloud_msg.width) * int(self.cloud_msg.height)
            print(f"[deskew] points={total} step={self.cloud_msg.point_step}")


def _setup_subscribers(viewer: MapViewer, map_state_topic: str, map_state_type: str, deskew_topic: str):
    if map_state_type == "heightmap":
        map_sub = ChannelSubscriber(map_state_topic, HeightMap_)
    elif map_state_type == "string":
        map_sub = ChannelSubscriber(map_state_topic, String_)
    elif map_state_type == "lidarstate":
        map_sub = ChannelSubscriber(map_state_topic, LidarState_)
    else:
        raise ValueError("map_state_type must be one of: heightmap, string, lidarstate")

    cloud_sub = ChannelSubscriber(deskew_topic, PointCloud2_)
    map_sub.Init(viewer.map_state_cb, 10)
    cloud_sub.Init(viewer.cloud_cb, 10)
    return map_sub, cloud_sub


def _try_visualize(viewer: MapViewer):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    plt.ion()
    fig1, ax1 = plt.subplots()
    fig2, ax2 = plt.subplots()
    map_im = None
    cloud_scatter = None

    while True:
        if viewer.map_state is not None and isinstance(viewer.map_state, HeightMap_):
            w = int(viewer.map_state.width)
            h = int(viewer.map_state.height)
            data = list(viewer.map_state.data)
            if data and w * h == len(data):
                grid = [data[i * w:(i + 1) * w] for i in range(h)]
                if map_im is None:
                    map_im = ax1.imshow(grid, origin="lower")
                    ax1.set_title("utlidar/map_state (HeightMap)")
                    fig1.colorbar(map_im, ax=ax1, fraction=0.046, pad=0.04)
                else:
                    map_im.set_data(grid)
                    map_im.autoscale()

        if viewer.cloud_msg is not None:
            pts = _extract_points_xy(viewer.cloud_msg, max_points=2000)
            if pts:
                xs, ys = zip(*pts)
                if cloud_scatter is None:
                    cloud_scatter = ax2.scatter(xs, ys, s=1)
                    ax2.set_title("utlidar/deskew cloud (xy)")
                    ax2.set_aspect("equal", "box")
                else:
                    cloud_scatter.set_offsets(list(zip(xs, ys)))

        plt.pause(0.05)


def main():
    parser = argparse.ArgumentParser(
        description="Subscribe to utlidar map_state + deskewed point cloud, with optional visualization."
    )
    parser.add_argument("iface", nargs="?", default="enp2s0", help="Robot network interface")
    parser.add_argument("--map-state-topic", default=TOPIC_MAP_STATE)
    parser.add_argument("--deskew-topic", default=TOPIC_DESKEW)
    parser.add_argument(
        "--map-state-type",
        default="heightmap",
        choices=["heightmap", "string", "lidarstate"],
        help="DDS type for map_state topic",
    )
    parser.add_argument("--no-viz", action="store_true", help="Disable matplotlib visualization")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    viewer = MapViewer(args.map_state_type)
    _setup_subscribers(viewer, args.map_state_topic, args.map_state_type, args.deskew_topic)

    if not args.no_viz:
        viz_ok = _try_visualize(viewer)
        if viz_ok is not False:
            return

    print("Matplotlib not available or disabled; printing status once per second.")
    while True:
        viewer.print_status()
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
