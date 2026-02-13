import argparse
import os
import sys
import time

import numpy as np

try:
    import cv2
except Exception as exc:
    print("OpenCV (cv2) is required for this script.")
    print(f"Import error: {exc}")
    sys.exit(1)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import config

if config.ROBOT == "g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

TOPIC_LOWSTATE = "rt/lowstate"


class ImuSubscriber:
    def __init__(self):
        self.last_msg = None
        self.last_ts = 0.0

    def cb(self, msg: LowState_):
        self.last_msg = msg
        self.last_ts = time.time()


def _format_vec(vec, ndigits=3):
    return [round(float(v), ndigits) for v in vec]


def run_imu(iface: str, domain_id: int):
    ChannelFactoryInitialize(domain_id, iface)

    imu = ImuSubscriber()
    sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
    sub.Init(imu.cb, 10)

    cv2.namedWindow("IMU")

    while True:
        canvas = np.zeros((360, 640, 3), dtype=np.uint8)
        y = 30
        dy = 28

        if imu.last_msg is None:
            cv2.putText(
                canvas,
                "Waiting for IMU (rt/lowstate)...",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 200, 200),
                2,
            )
        else:
            imu_state = imu.last_msg.imu_state
            q = _format_vec(imu_state.quaternion)
            g = _format_vec(imu_state.gyroscope)
            a = _format_vec(imu_state.accelerometer)

            cv2.putText(canvas, "IMU (rt/lowstate)", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            y += dy
            cv2.putText(canvas, f"Quat: {q}", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            y += dy
            cv2.putText(canvas, f"Gyro: {g} rad/s", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            y += dy
            cv2.putText(canvas, f"Accel: {a} m/s^2", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        cv2.imshow("IMU", canvas)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Show IMU data from lowstate.")
    parser.add_argument("--iface", default="lo", help="Network interface (use 'lo' for sim)")
    parser.add_argument("--domain_id", type=int, default=1, help="DDS domain id")
    args = parser.parse_args()

    run_imu(args.iface, args.domain_id)


if __name__ == "__main__":
    main()
