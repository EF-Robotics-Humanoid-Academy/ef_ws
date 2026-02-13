import argparse
import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient


last_imu_yaw = None
last_sport_pos = None
last_odom_pos = None


def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_cb(msg: LowState_):
    global last_imu_yaw
    last_imu_yaw = float(msg.imu_state.rpy[2])


def _sport_cb(msg: SportModeState_):
    global last_sport_pos
    last_sport_pos = [float(v) for v in msg.position]


def _odom_cb(msg: Odometry_):
    global last_odom_pos
    last_odom_pos = [
        float(msg.pose.pose.position.x),
        float(msg.pose.pose.position.y),
    ]


def _get_pose_xy():
    if last_odom_pos is not None:
        return last_odom_pos[0], last_odom_pos[1]
    if last_sport_pos is not None and len(last_sport_pos) >= 2:
        return last_sport_pos[0], last_sport_pos[1]
    return None


def _turn_to_delta(client, delta_yaw, yaw_rate, tick=0.05, timeout=8.0):
    if last_imu_yaw is None:
        raise RuntimeError("IMU yaw not available")
    start = last_imu_yaw
    target = abs(delta_yaw)
    end_time = time.time() + timeout
    while time.time() < end_time:
        if last_imu_yaw is None:
            time.sleep(tick)
            continue
        progress = abs(_wrap_angle(last_imu_yaw - start))
        if progress >= target:
            break
        client.Move(0.0, 0.0, yaw_rate)
        time.sleep(tick)
    client.StopMove()


def _walk_distance(client, speed, distance, tick=0.1, timeout=20.0):
    start = _get_pose_xy()
    if start is None:
        raise RuntimeError("No position source available (odom/sportstate)")
    end_time = time.time() + timeout
    while time.time() < end_time:
        pos = _get_pose_xy()
        if pos is None:
            time.sleep(tick)
            continue
        dx = pos[0] - start[0]
        dy = pos[1] - start[1]
        if math.hypot(dx, dy) >= distance:
            break
        client.Move(speed, 0.0, 0.0)
        time.sleep(tick)
    client.StopMove()


def main():
    parser = argparse.ArgumentParser(
        description="Go2 HL task: forward, 90 turn, wave, 180 opposite turn, wave, fast 90 turn, forward."
    )
    parser.add_argument("--iface", default="enp1s0")
    parser.add_argument("--speed", type=float, default=0.3, help="forward speed (m/s)")
    parser.add_argument("--forward-dist", type=float, default=1.0, help="forward distance (m)")
    parser.add_argument("--turn-dir", choices=["right", "left"], default="right")
    parser.add_argument("--turn-angle-deg", type=float, default=90.0)
    parser.add_argument("--yaw-rate", type=float, default=0.5, help="yaw rate (rad/s)")
    parser.add_argument("--run-yaw-rate", type=float, default=0.8, help="faster yaw rate (rad/s)")
    parser.add_argument("--free-walk", action="store_true", help="enable free walk before task")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    low_sub = ChannelSubscriber("rt/lowstate", LowState_)
    low_sub.Init(_yaw_cb, 10)
    sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sport_sub.Init(_sport_cb, 10)
    odom_sub = ChannelSubscriber("rt/odom", Odometry_)
    odom_sub.Init(_odom_cb, 10)

    client = SportClient()
    client.SetTimeout(5.0)
    client.Init()

    if args.free_walk:
        client.FreeWalk()
        time.sleep(0.2)

    yaw_sign = -1.0 if args.turn_dir == "right" else 1.0
    yaw_rate = yaw_sign * abs(args.yaw_rate)
    run_yaw_rate = yaw_sign * abs(args.run_yaw_rate)
    turn_duration = math.radians(args.turn_angle_deg) / max(1e-3, abs(args.yaw_rate))
    turn_180 = math.radians(180.0)

    try:
        time.sleep(0.2)
        _walk_distance(client, args.speed, args.forward_dist)
        _turn_to_delta(client, math.radians(args.turn_angle_deg), yaw_rate, timeout=turn_duration + 3.0)
        client.Hello()
        time.sleep(0.6)
        _turn_to_delta(client, turn_180, -yaw_rate, timeout=turn_duration * 2 + 3.0)
        client.Hello()
        time.sleep(0.6)
        _turn_to_delta(client, math.radians(args.turn_angle_deg), run_yaw_rate, timeout=turn_duration + 2.0)
        _walk_distance(client, args.speed, args.forward_dist)
    finally:
        client.StopMove()


if __name__ == "__main__":
    main()
