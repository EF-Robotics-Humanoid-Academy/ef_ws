from general_motion_retargeting.optitrack_vendor.NatNetClient import setup_optitrack
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
import threading
import argparse

def main(args):
    # Check if firewall is disabled on this machine
    print("Make sure to disable firewall on both machines:")
    print("On OptiTrack computer: Disable Windows Firewall")
    print("On this computer: sudo ufw disable")

    client = setup_optitrack(
        server_address=args.server_ip,
        client_address=args.client_ip,
        use_multicast=args.use_multicast,
    )

    # start a thread to client.run()
    thread = threading.Thread(target=client.run)
    thread.start()

    if not client:
        print("Failed to setup OptiTrack client")
        exit(1)

    print(f"OptiTrack client connected: {client.connected()}")
    print("Starting motion retargeting...")

    retarget = GMR(
            src_human="fbx",
            tgt_robot=args.robot,
            actual_human_height=args.human_height,
        )

    ESC_Press = False

    def on_viewer_key(keycode):
        nonlocal ESC_Press
        if keycode == 256:  # GLFW_KEY_ESCAPE
            ESC_Press = True

    viewer = RobotMotionViewer(
        robot_type=args.robot,
        keyboard_callback=on_viewer_key,
    )

    while not ESC_Press:
        frame = client.get_frame()
        frame_number = client.get_frame_number()
        qpos = retarget.retarget(frame)

        # print("=======================")
        # print(f"len(qpos) = {len(qpos)}")
        # for v in qpos:
        #     print(str(v))

        viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            rate_limit=False,
        )

    viewer.close()
    print("Viewer closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--server_ip", type=str, default="192.168.2.63")
    parser.add_argument("--client_ip", type=str, default="192.168.2.19")
    parser.add_argument("--use_multicast", default=False, action="store_true")
    parser.add_argument("--human_height", type=float, default=1.75, help="Actual human height (default: 1.75)")
    parser.add_argument("--robot", type=str, default="unitree_g1")
    args = parser.parse_args()
    main(args)
    