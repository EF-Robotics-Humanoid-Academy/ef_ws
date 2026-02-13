import argparse
import os
import time

import numpy as np

from unitree_sdk2py.core import channel as channel_module
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

# Disable CycloneDDS tracing to avoid config print crash
channel_module.ChannelConfigHasInterface = """<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS>
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface name=\"$__IF_NAME__$\" priority=\"default\" multicast=\"default\"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""
channel_module.ChannelConfigAutoDetermine = """<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS>
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""
os.environ.setdefault(
    "CYCLONEDDS_URI",
    "<CycloneDDS><Domain><Tracing><Category>none</Category></Tracing></Domain></CycloneDDS>",
)

TOPIC_LEFT_CMD = "rt/dex3/left/cmd"
TOPIC_RIGHT_CMD = "rt/dex3/right/cmd"
TOPIC_LEFT_STATE = "rt/dex3/left/state"
TOPIC_RIGHT_STATE = "rt/dex3/right/state"

JOINT_NAMES = [
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "middle_0",
    "middle_1",
    "index_0",
    "index_1",
]


class HandStatePrinter:
    def __init__(self, label, print_every=20):
        self.label = label
        self.count = 0
        self.print_every = print_every

    def cb(self, msg: HandState_):
        self.count += 1
        if self.count % self.print_every != 0:
            return
        q = [round(float(m.q), 3) for m in msg.motor_state]
        print(f"{self.label} q: {q}")


def build_cmd(targets, kp=1.5, kd=0.1, mode=1):
    msg = unitree_hg_msg_dds__HandCmd_()
    for i in range(7):
        cmd = msg.motor_cmd[i]
        cmd.mode = int(mode)
        cmd.tau = 0.0
        cmd.q = float(targets[i])
        cmd.dq = 0.0
        cmd.kp = float(kp)
        cmd.kd = float(kd)
    return msg


def run(side, pattern, open_q, close_q, rate_hz, kp, kd, iface, domain_id, print_state):
    if iface is not None:
        ChannelFactoryInitialize(domain_id, iface)
    else:
        ChannelFactoryInitialize(domain_id)

    pubs = []
    subs = []

    if side in ("left", "both"):
        pub = ChannelPublisher(TOPIC_LEFT_CMD, HandCmd_)
        pub.Init()
        pubs.append(pub)
        if print_state:
            sub = ChannelSubscriber(TOPIC_LEFT_STATE, HandState_)
            sub.Init(HandStatePrinter("L").cb, 10)
            subs.append(sub)

    if side in ("right", "both"):
        pub = ChannelPublisher(TOPIC_RIGHT_CMD, HandCmd_)
        pub.Init()
        pubs.append(pub)
        if print_state:
            sub = ChannelSubscriber(TOPIC_RIGHT_STATE, HandState_)
            sub.Init(HandStatePrinter("R").cb, 10)
            subs.append(sub)

    dt = 1.0 / rate_hz
    t = 0.0
    while True:
        if pattern == "open":
            targets = open_q
        elif pattern == "close":
            targets = close_q
        else:
            # sine between open and close
            phase = 0.5 * (1.0 + np.sin(2.0 * np.pi * 0.2 * t))
            targets = [o + (c - o) * phase for o, c in zip(open_q, close_q)]

        msg = build_cmd(targets, kp=kp, kd=kd)
        for pub in pubs:
            pub.Write(msg)

        t += dt
        time.sleep(dt)


def main():
    parser = argparse.ArgumentParser(description="Dex3-1 hand control (DDS).")
    parser.add_argument("--iface", default=None, help="Network interface (robot: enp1s0, sim: lo)")
    parser.add_argument("--domain_id", type=int, default=None, help="DDS domain id (robot: 0, sim: 1)")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--pattern", choices=["open", "close", "sine"], default="sine")
    parser.add_argument("--open", type=float, default=0.0, help="Open position (rad) for all joints")
    parser.add_argument("--close", type=float, default=0.7, help="Close position (rad) for all joints")
    parser.add_argument("--rate", type=float, default=50.0, help="Command rate (Hz)")
    parser.add_argument("--kp", type=float, default=1.5)
    parser.add_argument("--kd", type=float, default=0.1)
    parser.add_argument("--print_state", action="store_true")
    args = parser.parse_args()

    domain_id = 0 if args.domain_id is None else args.domain_id
    if args.iface is None:
        domain_id = 1 if args.domain_id is None else args.domain_id

    open_q = [args.open] * 7
    close_q = [args.close] * 7

    run(
        side=args.side,
        pattern=args.pattern,
        open_q=open_q,
        close_q=close_q,
        rate_hz=args.rate,
        kp=args.kp,
        kd=args.kd,
        iface=args.iface,
        domain_id=domain_id,
        print_state=args.print_state,
    )


if __name__ == "__main__":
    main()
