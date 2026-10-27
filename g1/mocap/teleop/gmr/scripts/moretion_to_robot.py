import argparse
import numpy as np
import time
import general_motion_retargeting.utils.lafan_vendor.utils as utils
from moretion import MotionToLafan1
from scipy.spatial.transform import Rotation as R
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer

LAFAN1_JOINT_NAMES = (
    "Hips",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToe",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToe",
    "Spine",
    "Spine1",
    "Spine2",
    "Neck",
    "Head",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
)


def process_frame(streamer):
    """
    Must return a dictionary with the following structure:
    {
        "Hips": (position, orientation),
        "Spine": (position, orientation),
        ...
    }
    """
    pos, rot = streamer.GetFrame()
    rot = rot[:, [3, 0, 1, 2]]  # (x, y, z, w) to (w, x, y, z)
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)

    result = {}
    for i, bone in enumerate(LAFAN1_JOINT_NAMES):
        position = pos[i] @ rotation_matrix.T * 0.01  # cm to m
        orientation = utils.quat_mul(rotation_quat, rot[i])
        result[bone] = [position, orientation]

    # Add modified foot pose for lafan1 format
    result["LeftFootMod"] = [result["LeftFoot"][0], result["LeftToe"][1]]
    result["RightFootMod"] = [result["RightFoot"][0], result["RightToe"][1]]

    return result


def moretion_retargeting(
    host: str = "192.168.2.92",
    port: int = 9999,
    use_udp: bool = False,
    ankle_angle: float = 85.0,
    robot: str = "unitree_g1",
    human_height: float = 1.75,
    motion_fps: int = 20,
    rate_limit: bool = False,
    record_video: bool = False,
    video_path=None,
):
    """
    连接 TCP 服务器, 并对 moretion 动作进行逐帧重定向

    host: 数据服务器IP地址
    port: 数据服务器端口
    robot: 重定向的机器人类型
    """

    ESC_Press = False

    def on_viewer_key(keycode):
        nonlocal ESC_Press
        if keycode == 256:  # GLFW_KEY_ESCAPE
            ESC_Press = True

    # Initialize the retargeting system
    retargeter = GMR(
        src_human="bvh_lafan1",
        tgt_robot=robot,
        actual_human_height=human_height,
    )

    robot_motion_viewer = RobotMotionViewer(
        robot_type=robot,
        motion_fps=motion_fps,
        transparent_robot=0,
        record_video=record_video,
        video_path=video_path,
        keyboard_callback=on_viewer_key,
        # video_width=2080,
        # video_height=1170
    )

    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds

    print(f"mocap_frame_rate: {motion_fps}")

    print(f"Connect to motion server {host}:{port}...")
    client = MotionToLafan1(host, port, use_udp, ankle_angle)
    time.sleep(0.5)
    print(
        f"Running status: {client.IsRunning()}\t"
        f"Recv count: {client.TotalCount()}\t"
        f"Valid count: {client.ValidCount()}\t"
        f"Queue size: {client.QueueLength()}"
    )
    while not ESC_Press:
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time

        # Update frame
        smplx_data = process_frame(client)

        # retarget
        qpos = retargeter.retarget(smplx_data)

        # visualize
        robot_motion_viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=rate_limit,
            follow_camera=True,
            # human_pos_offset=np.array([0.0, 0.0, 0.0])
        )

    client.Stop()
    robot_motion_viewer.close()
    print("motion client closed!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="连接 TCP/UDP 服务器, 接收运动数据并重定向到机器人"
    )
    parser.add_argument(
        "--server_ip",
        type=str,
        default="192.168.2.92",
        help="Moretion server IP address (default: 192.168.2.92)",
    )
    parser.add_argument(
        "--listen_port",
        type=int,
        default=9999,
        help="Moretion server port (default: 9999)",
    )
    parser.add_argument(
        "--use_udp",
        action="store_true",
        default=False,
        help="Whether to use udp to receive human motion (default: False)",
    )
    parser.add_argument(
        "--ankle_angle",
        type=float,
        default=85.0,
        help="Initial ankle angle to register moretion T-pose with Lafan1 (default: 85.0)",
    )
    parser.add_argument(
        "--robot",
        type=str,
        choices=[
            "unitree_g1",
            "unitree_g1_with_hands",
            "booster_t1",
            "stanford_toddy",
            "fourier_n1",
            "engineai_pm01",
            "pal_talos",
        ],
        default="unitree_g1",
        help="Target robot type (default: unitree_g1)",
    )
    parser.add_argument(
        "--human_height",
        type=float,
        default=1.75,
        help="Actual human height for retargeting (default: 1.75)",
    )
    parser.add_argument(
        "--motion_fps",
        default=30,
        type=int,
        help="Target FPS for retargeting (default: 30)",
    )
    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
        help="Whether to limit robot speed as human (default: False)",
    )
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
        help="Whether to record the video (default: False)",
    )
    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )
    args = parser.parse_args()

    moretion_retargeting(
        args.server_ip,
        args.listen_port,
        args.use_udp,
        args.ankle_angle,
        args.robot,
        args.human_height,
        args.motion_fps,
        args.rate_limit,
        args.record_video,
        args.video_path,
    )
