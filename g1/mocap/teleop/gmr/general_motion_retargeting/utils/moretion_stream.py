#!/usr/bin/env python3
"""
Moretion streaming data receiver and GMR adapter.

Receives moretion motion capture streaming data (JSON skeleton format) via
TCP (using ZMQ STREAM socket) or UDP (using raw socket), converts it to the
same per-frame format as ``load_moretion_file()`` for real-time retargeting
with ``GeneralMotionRetargeting``.

Streaming protocol (from 动捕软件串流端口协议.md):
  - JSON format: {"deviceList": [{bone: [x,y,z], ...}]}
  - Each bone has intrinsic XYZ Euler angles in degrees
    (bvhRotationOrder: "XYZ")
  - Root position: "coordinate" field, in meters, Y-up right-handed
  - Coordinate system: right-handed, X right, Y up, Z back

Bone name mapping (streaming -> BVH):
  crotch        -> Hips              (root rotation; BVH root is renamed to "Hips")
  coordinate    -> root position
  waistOne      -> Motion_Spine
  waistTwo      -> Motion_Spine1
  back          -> Motion_Spine2
  head          -> Motion_Head
  leftShoulder   -> Motion_LeftShoulder
  leftUpperArm   -> Motion_LeftArm
  leftLowerArm   -> Motion_LeftForeArm
  leftHand       -> Motion_LeftHand
  rightShoulder  -> Motion_RightShoulder
  rightUpperArm  -> Motion_RightArm
  rightLowerArm  -> Motion_RightForeArm
  rightHand      -> Motion_RightHand
  leftUpperLeg   -> Motion_LeftUpLeg
  leftLowerLeg   -> Motion_LeftLeg
  leftFoot       -> Motion_LeftFoot
  rightUpperLeg  -> Motion_RightUpLeg
  rightLowerLeg  -> Motion_RightLeg
  rightFoot      -> Motion_RightFoot

Bones in BVH hierarchy but absent from streaming (set to zero rotation):
  Motion_Neck, Motion_Neck1
"""

import json
import socket
import time
import numpy as np
from typing import Optional, Dict, Tuple, List

import zmq
from scipy.spatial.transform import Rotation as R

from general_motion_retargeting.utils.xsens_vendor.BVHParser import BVHParser
from general_motion_retargeting.utils.moretion import _euler_to_mujoco_quat
import general_motion_retargeting.utils.lafan_vendor.utils as utils

# --------------------------------------------------------------------------
# Bone name mapping: moretion streaming field -> BVH bone name
# --------------------------------------------------------------------------
STREAM_TO_BVH: Dict[str, str] = {
    "crotch": "Hips",
    "waistOne": "Motion_Spine",
    "waistTwo": "Motion_Spine1",
    "back": "Motion_Spine2",
    "head": "Motion_Head",
    "leftShoulder": "Motion_LeftShoulder",
    "leftUpperArm": "Motion_LeftArm",
    "leftLowerArm": "Motion_LeftForeArm",
    "leftHand": "Motion_LeftHand",
    "rightShoulder": "Motion_RightShoulder",
    "rightUpperArm": "Motion_RightArm",
    "rightLowerArm": "Motion_RightForeArm",
    "rightHand": "Motion_RightHand",
    "leftUpperLeg": "Motion_LeftUpLeg",
    "leftLowerLeg": "Motion_LeftLeg",
    "leftFoot": "Motion_LeftFoot",
    "rightUpperLeg": "Motion_RightUpLeg",
    "rightLowerLeg": "Motion_RightLeg",
    "rightFoot": "Motion_RightFoot",
}

# --------------------------------------------------------------------------
# Built-in skeleton (extracted from ue_body_hand.bvh with scale=0.01).
# Used when no reference BVH file is provided — the streaming protocol
# has a fixed bone hierarchy, so this static data is sufficient for FK.
# --------------------------------------------------------------------------
_DEFAULT_SKELETON_JSON = json.dumps({
    "names": [
        "Hips", "Motion_RightUpLeg", "Motion_RightLeg", "Motion_RightFoot",
        "Motion_RightFoot_end_site", "Motion_LeftUpLeg", "Motion_LeftLeg",
        "Motion_LeftFoot", "Motion_LeftFoot_end_site", "Motion_Spine",
        "Motion_Spine1", "Motion_Spine2", "Motion_Neck", "Motion_Neck1",
        "Motion_Head", "Motion_Head_end_site", "Motion_RightShoulder",
        "Motion_RightArm", "Motion_RightForeArm", "Motion_RightHand",
        "Motion_RightHandThumb1", "Motion_RightHandThumb2",
        "Motion_RightHandThumb3", "Motion_RightHandThumb3_end_site",
        "Motion_RightHandIndex", "Motion_RightHandIndex1",
        "Motion_RightHandIndex2", "Motion_RightHandIndex3",
        "Motion_RightHandIndex3_end_site", "Motion_RightHandMiddle",
        "Motion_RightHandMiddle1", "Motion_RightHandMiddle2",
        "Motion_RightHandMiddle3", "Motion_RightHandMiddle3_end_site",
        "Motion_RightHandRing", "Motion_RightHandRing1",
        "Motion_RightHandRing2", "Motion_RightHandRing3",
        "Motion_RightHandRing3_end_site", "Motion_RightHandPinky",
        "Motion_RightHandPinky1", "Motion_RightHandPinky2",
        "Motion_RightHandPinky3", "Motion_RightHandPinky3_end_site",
        "Motion_LeftShoulder", "Motion_LeftArm", "Motion_LeftForeArm",
        "Motion_LeftHand", "Motion_LeftHandThumb1",
        "Motion_LeftHandThumb2", "Motion_LeftHandThumb3",
        "Motion_LeftHandThumb3_end_site", "Motion_LeftHandIndex",
        "Motion_LeftHandIndex1", "Motion_LeftHandIndex2",
        "Motion_LeftHandIndex3", "Motion_LeftHandIndex3_end_site",
        "Motion_LeftHandMiddle", "Motion_LeftHandMiddle1",
        "Motion_LeftHandMiddle2", "Motion_LeftHandMiddle3",
        "Motion_LeftHandMiddle3_end_site", "Motion_LeftHandRing",
        "Motion_LeftHandRing1", "Motion_LeftHandRing2",
        "Motion_LeftHandRing3", "Motion_LeftHandRing3_end_site",
        "Motion_LeftHandPinky", "Motion_LeftHandPinky1",
        "Motion_LeftHandPinky2", "Motion_LeftHandPinky3",
        "Motion_LeftHandPinky3_end_site",
    ],
    "offsets": [
        [0.0, 0.0, 0.9143], [0.0, -0.0925, 0.0], [0.0, 0.0, -0.4187],
        [0.0, 0.0, -0.4187], [0.14, 0.0, -0.0769], [0.0, 0.0925, 0.0],
        [0.0, 0.0, -0.4187], [0.0, 0.0, -0.4187], [0.14, 0.0, -0.0769],
        [0.0, 0.0, 0.078176], [0.0, 0.0, 0.173104], [0.0, 0.0, 0.122848],
        [0.0, 0.0, 0.184272], [0.0, 0.0, 0.04735], [0.0, 0.0, 0.04735],
        [0.0, 0.0, 0.1595], [0.0, -0.027999, 0.128432],
        [0.0, -0.132001, 0.0], [0.0, -0.265, 0.0], [0.0, -0.26, 0.0],
        [0.02395, -0.01842, -0.00461], [0.0, -0.03682, 0.0],
        [0.0, -0.02559, 0.0], [0.0, -0.02413, 0.0],
        [0.01978, -0.03224, 0.00508], [0.00999, -0.05217, -0.00091],
        [0.0, -0.0362, 0.0], [0.0, -0.02052, 0.0], [0.0, -0.01805, 0.0],
        [0.00757, -0.03382, 0.00518], [0.00314, -0.05174, -0.00084],
        [0.0, -0.03949, 0.0], [0.0, -0.02476, 0.0], [0.0, -0.01975, 0.0],
        [-0.00129, -0.03366, 0.00538], [-0.00479, -0.04635, -0.00022],
        [0.0, -0.03442, 0.0], [0.0, -0.02388, 0.0], [0.0, -0.01857, 0.0],
        [-0.01202, -0.03161, 0.0047], [-0.01091, -0.04141, -0.00022],
        [0.0, -0.02757, 0.0], [0.0, -0.01742, 0.0], [0.0, -0.01645, 0.0],
        [0.0, 0.027999, 0.128432], [0.0, 0.132001, 0.0], [0.0, 0.265, 0.0],
        [0.0, 0.265, 0.0], [0.02395, 0.01842, -0.00461],
        [0.0, 0.03682, 0.0], [0.0, 0.02559, 0.0], [0.0, 0.02413, 0.0],
        [0.01978, 0.03224, 0.00508], [0.00999, 0.05217, -0.00091],
        [0.0, 0.0362, 0.0], [0.0, 0.02052, 0.0], [0.0, 0.01805, 0.0],
        [0.00757, 0.03382, 0.00518], [0.00314, 0.05174, -0.00084],
        [0.0, 0.03949, 0.0], [0.0, 0.02476, 0.0], [0.0, 0.01975, 0.0],
        [-0.00129, 0.03366, 0.00538], [-0.00479, 0.04635, -0.00022],
        [0.0, 0.03442, 0.0], [0.0, 0.02388, 0.0], [0.0, 0.01857, 0.0],
        [-0.01202, 0.03161, 0.0047], [-0.01091, 0.04141, -0.00022],
        [0.0, 0.02757, 0.0], [0.0, 0.01742, 0.0], [0.0, 0.01645, 0.0],
    ],
    "parents": [
        -1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 14, 11, 16, 17, 18,
        19, 20, 21, 22, 19, 24, 25, 26, 27, 19, 29, 30, 31, 32, 19, 34, 35,
        36, 37, 19, 39, 40, 41, 42, 11, 44, 45, 46, 47, 48, 49, 50, 47, 52,
        53, 54, 55, 47, 57, 58, 59, 60, 47, 62, 63, 64, 65, 47, 67, 68, 69,
        70,
    ],
})


class MoretionStreamReceiver:
    """Receive moretion streaming JSON data via TCP (ZMQ STREAM) or UDP.

    For TCP: uses ``zmq.STREAM`` socket type, which handles raw TCP
    connections without the ZMTP protocol.  Data is buffered and complete
    JSON objects are extracted via ``json.JSONDecoder.raw_decode``.

    For UDP: uses Python's ``socket`` module with ``SOCK_DGRAM``.  Each
    ``recvfrom`` call typically returns one complete JSON frame.

    Both transports drain queued data on each ``recv_frame()`` call and keep
    only the freshest frame; the number of frames discarded this way is
    accumulated in ``self.dropped_count`` (cumulative, never reset).
    """

    def __init__(self, host: str, port: int, use_udp: bool = False,
                 timeout: float = 5.0, device_id: Optional[str] = None):
        self.host = host
        self.port = port
        self.use_udp = use_udp
        self.timeout = timeout
        self.device_id = device_id  # if None, use first device

        self._zmq_ctx: Optional[zmq.Context] = None
        self._zmq_sock: Optional[zmq.Socket] = None
        self._udp_sock: Optional[socket.socket] = None
        self._tcp_buffer = b""
        self._decoder = json.JSONDecoder()
        self._connected = False

        # Cumulative count of frames discarded during draining (source faster
        # than the consumer).  Read by callers that want accurate drop stats.
        self.dropped_count: int = 0

    # -- public API --------------------------------------------------------

    def connect(self):
        """Establish connection to the moretion streaming server."""
        if self.use_udp:
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._udp_sock.settimeout(self.timeout)
            self._udp_sock.bind(("", self.port))
        else:
            self._zmq_ctx = zmq.Context.instance()
            self._zmq_sock = self._zmq_ctx.socket(zmq.STREAM)
            self._zmq_sock.setsockopt(zmq.RCVTIMEO, int(self.timeout * 1000))
            self._zmq_sock.connect(f"tcp://{self.host}:{self.port}")
        self._connected = True

    def is_running(self) -> bool:
        return self._connected

    def recv_frame(self) -> Optional[dict]:
        """Receive and parse one JSON frame.  Returns ``None`` on timeout."""
        if self.use_udp:
            return self._recv_udp()
        return self._recv_tcp()

    def close(self):
        """Close the socket and release resources."""
        if self._zmq_sock is not None:
            self._zmq_sock.close(linger=0)
            self._zmq_sock = None
        if self._udp_sock is not None:
            self._udp_sock.close()
            self._udp_sock = None
        self._connected = False

    # -- internals ---------------------------------------------------------

    def _recv_udp(self) -> Optional[dict]:
        """Receive the latest JSON frame, draining any pending packets."""
        try:
            data, _ = self._udp_sock.recvfrom(65536)
            obj = json.loads(data.decode("utf-8"))
        except (socket.timeout, json.JSONDecodeError, UnicodeDecodeError):
            return None
        # Drain: read any additional pending packets, keep only the latest
        self._udp_sock.setblocking(False)
        try:
            while True:
                try:
                    data, _ = self._udp_sock.recvfrom(65536)
                except (BlockingIOError, OSError):
                    break  # no more data
                try:
                    obj = json.loads(data.decode("utf-8"))
                    self.dropped_count += 1  # newer packet replaces older one
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue  # skip bad packet, keep draining
        finally:
            self._udp_sock.setblocking(True)
            self._udp_sock.settimeout(self.timeout)
        return obj

    def _recv_tcp(self) -> Optional[dict]:
        """Receive the latest complete JSON object from the ZMQ STREAM socket.

        Drains pending data to minimize latency: if multiple frames are
        buffered, only the most recent one is returned.
        """
        # Phase 1: Block until we have at least one complete frame
        obj = None
        while obj is None:
            obj = self._try_parse_buffer()
            if obj is not None:
                break
            try:
                msg = self._zmq_sock.recv_multipart()
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    return None  # timeout
                raise
            if len(msg) >= 2:
                self._tcp_buffer += msg[-1]

        # Phase 2: Drain both the bytes already buffered above and any newer
        # socket data. A single TCP chunk commonly contains several JSON
        # frames, so parsing only after another recv would return the oldest
        # frame and leave an ever-growing backlog in ``_tcp_buffer``.
        while True:
            while True:
                next_obj = self._try_parse_buffer()
                if next_obj is None:
                    break
                obj = next_obj
                self.dropped_count += 1  # newer frame replaces older one

            try:
                msg = self._zmq_sock.recv_multipart(zmq.NOBLOCK)
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    break  # no more data right now
                raise
            if len(msg) >= 2:
                self._tcp_buffer += msg[-1]

        return obj

    def _try_parse_buffer(self) -> Optional[dict]:
        """Attempt to decode one JSON object from the TCP buffer."""
        if not self._tcp_buffer:
            return None
        try:
            decoded = self._tcp_buffer.decode("utf-8")
            text = decoded.lstrip()
            obj, end = self._decoder.raw_decode(text)
            # ``raw_decode`` returns a character offset, while the receive
            # buffer is bytes. Re-encode the consumed prefix so non-ASCII
            # device metadata cannot shift the next JSON frame boundary.
            leading_chars = len(decoded) - len(text)
            consumed_text = decoded[:leading_chars + end]
            consumed_bytes = len(consumed_text.encode("utf-8"))
            self._tcp_buffer = self._tcp_buffer[consumed_bytes:]
            return obj
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None


class MoretionStreamToGMR:
    """Convert moretion streaming JSON data to GMR ``human_frame`` format.

    The bone hierarchy (offsets, parents, names) is needed for forward
    kinematics.  By default a built-in skeleton (extracted from
    ``ue_body_hand.bvh``) is used, so **no BVH file is required**.  Pass
    ``ref_bvh_file`` only if you need a custom skeleton.

    The output format matches ``load_moretion_file()``:
        ``{ bone_name: (position_np(3), quaternion_wxyz_np(4)) }``
    in the MuJoCo Z-up coordinate frame.
    """

    def __init__(self, ref_bvh_file: Optional[str] = None,
                 scale: float = 0.01, verbose: bool = True,
                 device_id: Optional[str] = None):
        if ref_bvh_file is not None:
            # ---- Parse reference BVH to get hierarchy ----
            parser = BVHParser(axis_order="zxy", scale=scale)
            with open(ref_bvh_file) as f:
                bvh_text = f.read()
            bvh_text = bvh_text.replace("ROOT Motion_Hips", "ROOT Hips", 1)
            parser.parse(bvh_text, start=0, end=1, reset_to_zero=False)
            self._parser = parser
            names = parser.names
            offsets = np.array(parser.offsets)
            parents = np.array(parser.parents, dtype=int)
        else:
            # ---- Use built-in skeleton (no BVH file needed) ----
            skel = json.loads(_DEFAULT_SKELETON_JSON)
            self._parser = None
            names = skel["names"]
            offsets = np.array(skel["offsets"], dtype=float)
            parents = np.array(skel["parents"], dtype=int)

        self.names: List[str] = names
        self.offsets = offsets        # (N, 3) MuJoCo frame
        self.parents = parents
        self.scale = scale
        self.verbose = verbose
        self.device_id = str(device_id) if device_id is not None else None

        self.name_to_idx = {n: i for i, n in enumerate(self.names)}
        self.bvh_to_stream = {v: k for k, v in STREAM_TO_BVH.items()}

        # Bones in BVH that have no streaming equivalent (excluding end sites)
        self.missing_bones = [
            n for n in self.names
            if n != "Hips" and n not in self.bvh_to_stream
            and not n.endswith("_end_site")
        ]

        # Yaw compensation state (captured from first frame)
        self._yaw_comp_quat: Optional[np.ndarray] = None  # wxyz
        self._yaw_comp_rot: Optional[np.ndarray] = None   # 3x3 matrix
        self._yaw_init = False

        # Previous frame quaternions for sign-continuity checking
        self._prev_quats: Optional[np.ndarray] = None

        if verbose:
            print(f"[MoretionStreamToGMR] Reference BVH: {len(self.names)} bones")
            if self.missing_bones:
                print(f"  Bones without streaming data (set to zero): "
                      f"{self.missing_bones}")

    # ------------------------------------------------------------------

    def convert_frame(self, json_data: dict) -> dict:
        """Convert one streaming JSON frame to a GMR ``human_frame`` dict.

        Parameters
        ----------
        json_data : dict
            Parsed JSON from the streaming server, e.g.
            ``{"deviceList": [{...}]}``.

        Returns
        -------
        dict
            ``{ bone_name: (position, quaternion_wxyz) }`` in MuJoCo frame.
        """
        devices = json_data.get("deviceList", [])
        if not devices:
            raise ValueError("No devices in streaming data")
        if self.device_id is None:
            device = devices[0]
        else:
            device = next(
                (
                    item
                    for item in devices
                    if str(item.get("deviceId")) == self.device_id
                ),
                None,
            )
            if device is None:
                raise ValueError(
                    f"Device ID {self.device_id!r} not found in streaming data"
                )

        N = len(self.names)
        rotations = np.zeros((1, N, 3))  # stored as [Z, X, Y]

        # Fill euler angles from streaming data
        for stream_name, bvh_name in STREAM_TO_BVH.items():
            if stream_name not in device:
                continue
            idx = self.name_to_idx.get(bvh_name)
            if idx is None:
                continue
            euler_xyz = np.array(device[stream_name], dtype=float)  # [X, Y, Z]
            # Store as [Z, X, Y] to match BVHParser convention
            rotations[0, idx] = [euler_xyz[2], euler_xyz[0], euler_xyz[1]]

        # Bones absent from streaming remain zero (already initialized)

        # Convert euler to MuJoCo-frame quaternions
        quats = _euler_to_mujoco_quat(rotations)  # (1, N, 4) wxyz

        # Sign-continuity: flip quaternions that disagree with previous frame
        quats = self._ensure_continuity(quats)

        # Build positions array (all bones start at their offset)
        positions = self.offsets[np.newaxis].copy()  # (1, N, 3)

        # Root position from streaming "coordinate" (meters, BVH Y-up)
        coord = np.array(device.get("coordinate", [0.0, 0.0, 0.0]),
                         dtype=float)
        # Convert BVH [X, Y, Z] -> MuJoCo [Z, X, Y] (axis_idx = [2, 0, 1])
        # Streaming coordinate is already in meters — no scale needed
        positions[0, 0] = np.array([coord[2], coord[0], coord[1]])

        # ---- Root yaw compensation (fixed reference from first frame) ----
        self._apply_root_compensation(quats, positions)

        # ---- Forward kinematics ----
        gr, gp = utils.quat_fk(quats, positions, self.parents)

        # ---- Build frame dict ----
        # gp/gr are fresh arrays from quat_fk; downstream (retarget) only
        # reads them (no in-place modification), so views are safe and
        # .copy() would only waste allocations.
        result: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for i, bone in enumerate(self.names):
            result[bone] = (gp[0, i], gr[0, i])

        # Pseudo-bones for ankle IK targets (same as load_moretion_file)
        result["LeftFootMod"] = (
            np.array(result["Motion_LeftFoot"][0]),
            result["Motion_LeftFoot"][1],
        )
        result["RightFootMod"] = (
            np.array(result["Motion_RightFoot"][0]),
            result["Motion_RightFoot"][1],
        )

        return result

    def get_human_height(self, frame: dict) -> float:
        """Estimate human height from a converted frame (metres)."""
        try:
            head_z = frame["Motion_Head_end_site"][0][2]
            lfoot_z = frame["Motion_LeftFoot_end_site"][0][2]
            rfoot_z = frame["Motion_RightFoot_end_site"][0][2]
            return float(head_z - min(lfoot_z, rfoot_z))
        except KeyError:
            return 1.75

    # ------------------------------------------------------------------
    #  Private helpers
    # ------------------------------------------------------------------

    def _ensure_continuity(self, quats: np.ndarray) -> np.ndarray:
        """Flip quaternion signs to stay consistent with the previous frame."""
        if self._prev_quats is not None:
            dot = np.sum(quats[0] * self._prev_quats[0], axis=-1)
            mask = (dot < 0).astype(float)[:, np.newaxis]
            quats[0] = mask * (-quats[0]) + (1.0 - mask) * quats[0]
        self._prev_quats = quats.copy()
        return quats

    def _apply_root_compensation(self, quats: np.ndarray,
                                  positions: np.ndarray):
        """Remove initial yaw from root rotation and position.

        Mirrors ``BVHParser.compensate_z_rotation`` and
        ``compensate_displacements``, but uses the **first received frame**
        as the fixed yaw reference (instead of per-frame).
        """
        root_q = quats[0, 0] / np.linalg.norm(quats[0, 0])
        if not self._yaw_init:
            # Extract yaw using 2*atan2(z, w) — matches the training-side
            # get_heading / get_heading_q in torch_transform.py and
            # BVHParser.compensate_z_rotation.  This method zeros the x,y
            # quaternion components (equivalently uses 2*atan2(z,w)) and is
            # pitch-invariant: for q = Ry(p)*Rz(y) it returns the exact yaw y
            # regardless of pitch p.  The alternative calc_heading (rotating
            # [1,0,0] and taking atan2(y,x)) is contaminated by pitch.
            w, x, y, z = root_q
            theta = 2.0 * np.arctan2(z, w)
            # Compensation rotation: -theta about Z
            self._yaw_comp_quat = np.array([
                np.cos(-theta / 2), 0.0, 0.0, np.sin(-theta / 2)
            ])
            self._yaw_comp_rot = np.array([
                [np.cos(-theta), -np.sin(-theta), 0.0],
                [np.sin(-theta),  np.cos(-theta), 0.0],
                [0.0,              0.0,            1.0],
            ])
            # Also zero out the initial XZ position
            self._init_pos_xz = positions[0, 0, 0:2].copy()
            self._yaw_init = True

        # Remove initial XZ offset
        positions[0, 0, 0:2] -= self._init_pos_xz

        # Rotate root position by -yaw (compensate displacement)
        positions[0, 0] = self._yaw_comp_rot @ positions[0, 0]

        # Remove yaw from root quaternion: q_new = q_comp * q (Hamilton product)
        # Matches BVHParser.compensate_z_rotation's quaternion_multiply(q_comp, q)
        qc = self._yaw_comp_quat
        q = root_q
        w = qc[0]*q[0] - qc[1]*q[1] - qc[2]*q[2] - qc[3]*q[3]
        x = qc[0]*q[1] + qc[1]*q[0] + qc[2]*q[3] - qc[3]*q[2]
        y = qc[0]*q[2] - qc[1]*q[3] + qc[2]*q[0] + qc[3]*q[1]
        z = qc[0]*q[3] + qc[1]*q[2] - qc[2]*q[1] + qc[3]*q[0]
        quats[0, 0] = np.array([w, x, y, z])
        quats[0, 0] /= np.linalg.norm(quats[0, 0])
