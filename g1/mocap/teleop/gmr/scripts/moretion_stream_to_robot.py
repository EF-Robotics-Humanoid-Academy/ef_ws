#!/usr/bin/env python3
"""
Moretion Live Streaming to Robot Retargeting.

Connects to a moretion motion-capture streaming server (TCP via ZMQ STREAM
or UDP via raw socket), receives per-bone euler-angle JSON frames, converts
them to GMR ``human_frame`` format, and retargets to a robot in real time.

Usage examples:
    # TCP (default)
    python scripts/moretion_stream_to_robot.py \\
        --server_ip 192.168.2.92 --port 9999

    # UDP
    python scripts/moretion_stream_to_robot.py \\
        --server_ip 192.168.2.92 --port 9999 --use_udp

    # Save trajectory to pickle
    python scripts/moretion_stream_to_robot.py --save_path output/motion.pkl
"""

import argparse
import os
import signal
import sys
import time

import numpy as np
from rich import print

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.moretion_stream import (
    MoretionStreamReceiver,
    MoretionStreamToGMR,
)

# Global flag for graceful shutdown
g_running = True


def signal_handler(signum, frame):
    global g_running
    print(f"\nReceived signal {signum}, shutting down...")
    g_running = False


if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    parser = argparse.ArgumentParser(
        description="Moretion live streaming to robot retargeting",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ---- Streaming connection ----
    parser.add_argument(
        "--server_ip",
        type=str,
        default="192.168.2.92",
        help="Moretion streaming server IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9999,
        help="Moretion streaming server port",
    )
    parser.add_argument(
        "--use_udp",
        action="store_true",
        default=False,
        help="Use UDP instead of TCP to receive streaming data",
    )
    parser.add_argument(
        "--device_id",
        type=str,
        default=None,
        help="Device ID to select from deviceList (default: first device)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Receive timeout in seconds",
    )

    # ---- Reference BVH (for bone hierarchy only) ----
    parser.add_argument(
        "--ref_bvh",
        type=str,
        default=None,
        help="Reference BVH for custom skeleton (default: built-in moretion skeleton)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=0.01,
        help="BVH scale (cm->m for offsets; streaming coordinate is already in metres)",
    )

    # ---- Retargeting ----
    parser.add_argument(
        "--robot",
        choices=[
            "unitree_g1",
            "unitree_g1_with_hands",
            "unitree_h1_2",
            "Q1",
            "X1",
        ],
        default="unitree_g1",
    )
    parser.add_argument(
        "--human_height",
        type=float,
        default=None,
        help="Actual human height in metres (auto-detected from first frame if omitted)",
    )
    parser.add_argument(
        "--motion_fps",
        type=int,
        default=60,
        help="Target FPS for the robot motion viewer",
    )
    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=True,
        help="Limit robot joint speed to human-like rates",
    )

    # ---- Visualization / output ----
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/moretion_stream.mp4",
    )
    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the retargeted robot motion as a pickle file",
    )

    args = parser.parse_args()

    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []

    # ------------------------------------------------------------------
    # 1. Initialize stream receiver and frame converter
    # ------------------------------------------------------------------
    print("[1/4] Initializing moretion stream receiver...")
    receiver = MoretionStreamReceiver(
        host=args.server_ip,
        port=args.port,
        use_udp=args.use_udp,
        timeout=args.timeout,
        device_id=args.device_id,
    )

    print("[2/4] Initializing frame converter...")
    converter = MoretionStreamToGMR(
        ref_bvh_file=args.ref_bvh,
        scale=args.scale,
        verbose=True,
        device_id=args.device_id,
    )

    # ------------------------------------------------------------------
    # 3. Connect and wait for first frame to determine human height
    # ------------------------------------------------------------------
    print(f"[3/4] Connecting to {args.server_ip}:{args.port} "
          f"({'UDP' if args.use_udp else 'TCP'})...")
    receiver.connect()
    time.sleep(0.5)

    print("Waiting for first frame...")
    first_frame = None
    while g_running and first_frame is None:
        raw = receiver.recv_frame()
        if raw is None:
            continue
        try:
            first_frame = converter.convert_frame(raw)
        except Exception as e:
            print(f"  First frame conversion failed: {e}, retrying...")
            continue

    if not g_running:
        receiver.close()
        sys.exit(0)

    # Determine human height
    if args.human_height is not None:
        human_height = args.human_height
    else:
        human_height = converter.get_human_height(first_frame)
    print(f"  Human height: {human_height:.3f} m")

    # ------------------------------------------------------------------
    # 4. Initialize retargeter and viewer
    # ------------------------------------------------------------------
    print("[4/4] Initializing retargeter and viewer...")
    retargeter = GMR(
        src_human="bvh_moretion",
        tgt_robot=args.robot,
        actual_human_height=human_height,
        solver="daqp",
        damping=1.0,
        use_velocity_limit=True,
    )

    robot_motion_viewer = RobotMotionViewer(
        robot_type=args.robot,
        motion_fps=args.motion_fps,
        transparent_robot=0,
        record_video=args.record_video,
        video_path=args.video_path,
    )

    print(f"mocap_frame_rate: {args.motion_fps}")
    print("Starting live retargeting... Press Ctrl+C to stop\n")

    # FPS measurement
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0

    total_frames = 0
    dropped_frames = 0
    last_valid_qpos = None
    last_valid_human_frame = None

    # Retarget the first frame we already received
    try:
        qpos = retargeter.retarget(first_frame)
        last_valid_qpos = qpos.copy()
        last_valid_human_frame = retargeter.scaled_human_data
        robot_motion_viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=args.rate_limit,
            follow_camera=True,
        )
        if args.save_path is not None:
            qpos_list.append(qpos)
        total_frames += 1
    except Exception as e:
        print(f"First frame retargeting failed: {e}")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    try:
        while g_running:
            raw = receiver.recv_frame()
            if raw is None:
                dropped_frames += 1
                # Show last valid pose if available
                if last_valid_qpos is not None:
                    robot_motion_viewer.step(
                        root_pos=last_valid_qpos[:3],
                        root_rot=last_valid_qpos[3:7],
                        dof_pos=last_valid_qpos[7:],
                        human_motion_data=last_valid_human_frame,
                        rate_limit=args.rate_limit,
                        follow_camera=True,
                    )
                time.sleep(0.001)
                continue

            # Convert JSON frame to GMR human_frame
            try:
                human_frame = converter.convert_frame(raw)
            except Exception as e:
                print(f"Frame conversion failed: {e}")
                dropped_frames += 1
                continue

            # Retarget
            try:
                qpos = retargeter.retarget(human_frame)
            except Exception as e:
                print(f"Retargeting failed: {e}")
                dropped_frames += 1
                continue

            last_valid_qpos = qpos.copy()
            last_valid_human_frame = retargeter.scaled_human_data
            total_frames += 1

            # Visualize
            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retargeter.scaled_human_data,
                rate_limit=args.rate_limit,
                follow_camera=True,
            )

            if args.save_path is not None:
                qpos_list.append(qpos)

            # FPS measurement
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time >= fps_display_interval:
                actual_fps = fps_counter / (current_time - fps_start_time)
                print(
                    f"FPS: {actual_fps:.1f} | "
                    f"Retargeted: {total_frames} | "
                    f"Dropped: {dropped_frames}"
                )
                fps_counter = 0
                fps_start_time = current_time

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        receiver.close()

        # Save trajectory
        if args.save_path is not None and qpos_list:
            import pickle

            root_pos = np.array([qpos[:3] for qpos in qpos_list])
            # save from wxyz to xyzw
            root_rot = np.array([qpos[3:7][[1, 2, 3, 0]] for qpos in qpos_list])
            dof_pos = np.array([qpos[7:] for qpos in qpos_list])

            motion_data = {
                "fps": args.motion_fps,
                "root_pos": root_pos,
                "root_rot": root_rot,
                "dof_pos": dof_pos,
                "local_body_pos": None,
                "link_body_list": None,
            }
            with open(args.save_path, "wb") as f:
                pickle.dump(motion_data, f)
            print(f"Saved to {args.save_path}")

        # Print final stats
        print(f"\nTotal retargeted: {total_frames}")
        print(f"Total dropped: {dropped_frames}")

        robot_motion_viewer.close()
