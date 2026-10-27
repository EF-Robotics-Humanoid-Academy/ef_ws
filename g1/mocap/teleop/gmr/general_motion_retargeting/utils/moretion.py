import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.xsens_vendor.BVHParser import (
    BVHParser,
    Anim,
    remove_quat_discontinuities,
)
import numpy as np
from scipy.spatial.transform import Rotation as R

# Coordinate transform quaternion: BVH Y-up -> MuJoCo Z-up.
# Maps BVH [X, Y, Z] -> MuJoCo [Z, X, Y] (i.e. BVH_X->MuJoCo_Y,
# BVH_Y->MuJoCo_Z, BVH_Z->MuJoCo_X).  This is a 120° rotation about
# the [1, 1, 1] axis, represented as wxyz quaternion.
_Q_T = R.from_quat([0.5, 0.5, 0.5, 0.5], scalar_first=True)
_Q_T_INV = _Q_T.inv()


def _euler_to_mujoco_quat(rotations_zyx):
    """
    Convert moretion BVH euler angles to MuJoCo-frame quaternions.

    Moretion BVH uses **XYZ intrinsic** rotation order.
    BVHParser stores the reordered values as [Z, X, Y] (after axis_idx).
    For each joint we:
      1. Recover the original [X, Y, Z] euler values.
      2. Convert to a BVH-frame quaternion via intrinsic XYZ.
      3. Apply the coordinate-system transform via quaternion conjugation:
             q_mujoco = q_T * q_bvh * q_T^{-1}

    SciPy denotes intrinsic rotations with uppercase axis names. This differs
    from BVHParser's lowercase ``xyz`` (extrinsic), which is not correct for
    Moretion's intrinsic stream/BVH convention.

    Parameters
    ----------
    rotations_zyx : np.ndarray, shape (..., 3)
        Euler angles as stored by BVHParser: [Z, X, Y] in degrees.

    Returns
    -------
    np.ndarray, shape (..., 4)
        Quaternions in wxyz format, in the MuJoCo coordinate frame.
    """
    # Recover original [X, Y, Z] from stored [Z, X, Y]
    orig_shape = rotations_zyx.shape
    flat = rotations_zyx.reshape(-1, 3)
    euler_xyz = np.empty_like(flat)
    euler_xyz[:, 0] = flat[:, 1]  # X
    euler_xyz[:, 1] = flat[:, 2]  # Y
    euler_xyz[:, 2] = flat[:, 0]  # Z

    # Convert to BVH-frame quaternion (intrinsic XYZ; SciPy uses uppercase).
    r_bvh = R.from_euler("XYZ", euler_xyz, degrees=True)

    # Apply coordinate transform: q_mujoco = q_T * q_bvh * q_T^{-1}
    r_mujoco = _Q_T * r_bvh * _Q_T_INV
    return r_mujoco.as_quat(scalar_first=True).reshape(*orig_shape[:-1], 4)


def load_moretion_file(args):
    """
    Load a moretion-format BVH file and return per-frame bone data.

    Moretion BVH files use bone names prefixed with "Motion_" (e.g.
    "Motion_Hips", "Motion_RightUpLeg") and **XYZ intrinsic** rotation
    channels.

    Returns a list of frames, each frame is a dict:
        { bone_name: (position, orientation_quaternion) }

    The root bone is renamed to "Hips" during parsing so that
    BVHParser picks up the root translation from the MOTION data.
    """
    parser = BVHParser(axis_order="zxy", scale=args.scale)
    with open(args.bvh_file, "r") as f:
        bvh_text = f.read()

    # BVHParser only updates root translation when node.name == "Hips".
    # Moretion names the root "Motion_Hips", so rename it for parsing.
    bvh_text = bvh_text.replace("ROOT Motion_Hips", "ROOT Hips", 1)

    # parse() returns raw euler angles (stored as [Z, X, Y]) and positions.
    rotations, positions = parser.parse(
        bvh_text, start=args.start, end=args.end, reset_to_zero=args.reset_to_zero
    )

    # ------------------------------------------------------------------
    # Correct Euler-to-quaternion conversion for Moretion's intrinsic XYZ
    # order (replaces BVHParser._MOTION_data_post_processing which uses
    # extrinsic xyz, which is wrong for Moretion's intrinsic XYZ convention).
    # ------------------------------------------------------------------
    quats = _euler_to_mujoco_quat(rotations)  # (fnum, N, 4) wxyz, MuJoCo frame
    quats = remove_quat_discontinuities(quats)

    offsets = np.array(parser.offsets)  # (N, 3) already in MuJoCo frame
    parents = np.array(parser.parents, dtype=int)  # (N,)
    positions = np.copy(parser.positions)  # (fnum, N, 3) already in MuJoCo frame

    # Replicate BVHParser's reset_to_zero logic (always enabled, matching
    # the original moretion loader which hardcoded reset_to_zero=True).
    positions[:, 0][:, 0:2] -= positions[:, 0][0, 0:2]
    positions[:, 0] = parser.compensate_displacements(quats[:, 0], positions[:, 0])
    quats[:, 0] = parser.compensate_z_rotation(quats[:, 0])

    anim = Anim(quats, positions, offsets, parents, parser.names)
    global_data = utils.quat_fk(anim.quats, anim.pos, anim.parents)

    frames = []
    for frame in range(anim.pos.shape[0]):
        result = {}
        for i, bone in enumerate(anim.bones):
            orientation = global_data[0][frame, i]
            position = global_data[1][frame, i]
            result[bone] = (position, orientation)

        # Reuse the ankle joint as the foot target, consistent with the
        # xsens LeftFootMod / RightFootMod convention.
        result["LeftFootMod"] = (
            np.array(result["Motion_LeftFoot"][0]),
            result["Motion_LeftFoot"][1],
        )
        result["RightFootMod"] = (
            np.array(result["Motion_RightFoot"][0]),
            result["Motion_RightFoot"][1],
        )
        frames.append(result)

    human_height = result["Motion_Head_end_site"][0][2] - min(
        result["Motion_LeftFoot_end_site"][0][2],
        result["Motion_RightFoot_end_site"][0][2],
    )
    return frames, human_height, parser.frame_time
