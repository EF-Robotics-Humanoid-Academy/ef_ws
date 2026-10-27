import os
import time
import mujoco as mj
import mujoco.viewer as mjv
import imageio
from scipy.spatial.transform import Rotation as R
from general_motion_retargeting import ROBOT_XML_DICT, ROBOT_BASE_DICT, VIEWER_CAM_DISTANCE_DICT
from loop_rate_limiters import RateLimiter
import numpy as np
from rich import print

GLFW_KEY_V = 86
GLFW_KEY_B = 66

GROUND_STYLE_PRESETS = [
    {"name": "Default Checker"},
    {"name": "Dark Reflective Grid"},
    {"name": "Hidden"},
]

FLOOR_VIS_GROUP = 3


def draw_frame(
    pos,
    mat,
    v,
    size,
    joint_name=None,
    orientation_correction=R.from_euler("xyz", [0, 0, 0]),
    pos_offset=np.array([0, 0, 0]),
):
    rgba_list = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    for i in range(3):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=[0.01, 0.01, 0.01],
            pos=pos + pos_offset,
            mat=mat.flatten(),
            rgba=rgba_list[i],
        )
        if joint_name is not None:
            geom.label = joint_name  # 这里赋名字
        fix = orientation_correction.as_matrix()
        mj.mjv_connector(
            v.user_scn.geoms[v.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=0.005,
            from_=pos + pos_offset,
            to=pos + pos_offset + size * (mat @ fix)[:, i],
        )
        v.user_scn.ngeom += 1

class RobotMotionViewer:
    def __init__(self,
                robot_type,
                camera_follow=True,
                motion_fps=30,
                transparent_robot=0,
                # video recording
                record_video=False,
                video_path=None,
                video_width=640,
                video_height=480,
                keyboard_callback=None,
                ):
        
        self.robot_type = robot_type
        self.xml_path = ROBOT_XML_DICT[robot_type]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.robot_base = ROBOT_BASE_DICT[robot_type]
        self.viewer_cam_distance = VIEWER_CAM_DISTANCE_DICT[robot_type]
        mj.mj_step(self.model, self.data)
        
        self.motion_fps = motion_fps
        self.rate_limiter = RateLimiter(frequency=self.motion_fps, warn=False)
        self.record_video = record_video

        self._camera_follow = camera_follow
        self._ground_style_index = 0
        self._floor_geom_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "floor")
        self._floor_original_matid = None
        self._floor_original_rgba = None
        self._floor_original_group = None

        self._dark_grid_matid = self._find_dark_grid_matid()

        if self._floor_geom_id >= 0:
            self._floor_original_matid = int(self.model.geom_matid[self._floor_geom_id])
            self._floor_original_rgba = self.model.geom_rgba[self._floor_geom_id].copy()
            self._floor_original_group = int(self.model.geom_group[self._floor_geom_id])
            self.model.geom_group[self._floor_geom_id] = FLOOR_VIS_GROUP

        self._pending_flag_restores = []

        combined_callback = self._make_key_callback(keyboard_callback)

        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False, 
            key_callback=combined_callback
            )      

        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = transparent_robot

        self._apply_soft_lighting()

        if self._floor_geom_id >= 0:
            geomgroup = list(self.viewer.opt.geomgroup)
            if len(geomgroup) <= FLOOR_VIS_GROUP:
                geomgroup += [0] * (FLOOR_VIS_GROUP + 1 - len(geomgroup))
            geomgroup[FLOOR_VIS_GROUP] = 1
            self.viewer.opt.geomgroup = geomgroup

        print("[Viewer Shortcuts]")
        print("  V - Toggle camera follow / fixed mode")
        print("  B - Cycle ground material style")
        
        if self.record_video:
            assert video_path is not None, "Please provide video path for recording"
            self.video_path = video_path
            video_dir = os.path.dirname(self.video_path)
            
            if not os.path.exists(video_dir):
                os.makedirs(video_dir)
            self.mp4_writer = imageio.get_writer(self.video_path, fps=self.motion_fps)
            print(f"Recording video to {self.video_path}")
            
            # Initialize renderer for video recording
            self.renderer = mj.Renderer(self.model, height=video_height, width=video_width)

    def _find_dark_grid_matid(self):
        for i in range(self.model.nmat):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_MATERIAL, i)
            if name == "MatPlane":
                return i
        return None

    def _apply_soft_lighting(self):
        self.model.vis.headlight.ambient = [0.07, 0.07, 0.07]
        self.model.vis.headlight.diffuse = [0.12, 0.12, 0.12]
        self.model.vis.headlight.specular = [0.0, 0.0, 0.0]

        for i in range(self.model.nlight):
            self.model.light_ambient[i] = [0.08, 0.08, 0.1]
            self.model.light_diffuse[i] = [0.13, 0.13, 0.13]
            self.model.light_specular[i] = [0.0, 0.0, 0.0]
            self.model.light_type[i] = 1
            self.model.light_cutoff[i] = 90
            self.model.light_exponent[i] = 1

        for i in range(self.model.nmat):
            self.model.mat_reflectance[i] = 0.0
            self.model.mat_shininess[i] = 0.05
            self.model.mat_specular[i] = 0.05

    def _make_key_callback(self, user_callback):
        def callback(keycode):
            self._on_key_press(keycode)
            if user_callback is not None:
                user_callback(keycode)
        return callback

    def _on_key_press(self, keycode):
        if keycode == GLFW_KEY_V:
            self._camera_follow = not self._camera_follow
            mode = "FOLLOW" if self._camera_follow else "FIXED"
            print(f"Camera: {mode} mode")
            self._pending_flag_restores.append(
                ("vis", mj.mjtVisFlag.mjVIS_TENDON, self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TENDON]))

        elif keycode == GLFW_KEY_B:
            self._ground_style_index = (self._ground_style_index + 1) % len(GROUND_STYLE_PRESETS)
            preset = GROUND_STYLE_PRESETS[self._ground_style_index]
            self._apply_ground_style(preset)
            print(f"Ground: {preset['name']}")
            self._pending_flag_restores.append(
                ("vis", mj.mjtVisFlag.mjVIS_PERTFORCE, self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_PERTFORCE]))

    def _apply_ground_style(self, preset):
        if self._floor_geom_id < 0:
            return

        name = preset["name"]

        if name == "Hidden":
            geomgroup = list(self.viewer.opt.geomgroup)
            if len(geomgroup) > FLOOR_VIS_GROUP:
                geomgroup[FLOOR_VIS_GROUP] = 0
                self.viewer.opt.geomgroup = geomgroup
        else:
            geomgroup = list(self.viewer.opt.geomgroup)
            if len(geomgroup) <= FLOOR_VIS_GROUP:
                geomgroup += [0] * (FLOOR_VIS_GROUP + 1 - len(geomgroup))
            geomgroup[FLOOR_VIS_GROUP] = 1
            self.viewer.opt.geomgroup = geomgroup

            if name == "Default Checker":
                self.model.geom_matid[self._floor_geom_id] = self._floor_original_matid
                self.model.geom_rgba[self._floor_geom_id] = self._floor_original_rgba
            elif name == "Dark Reflective Grid":
                if self._dark_grid_matid is None:
                    self.model.geom_matid[self._floor_geom_id] = -1
                    self.model.geom_rgba[self._floor_geom_id] = np.array([0.2, 0.2, 0.2, 1.0])
                else:
                    self.model.geom_matid[self._floor_geom_id] = self._dark_grid_matid
                    self.model.geom_rgba[self._floor_geom_id] = np.array([1.0, 1.0, 1.0, 1.0])
            else:
                self.model.geom_matid[self._floor_geom_id] = -1
                self.model.geom_rgba[self._floor_geom_id] = preset["rgba"]
        
    def step(self, 
            # robot data
            root_pos, root_rot, dof_pos, 
            # human data
            human_motion_data=None, 
            show_human_body_name=False,
            # scale for human point visualization
            human_point_scale=0.1,
            # human pos offset add for visualization    
            human_pos_offset=np.array([0.0, 0.0, 0]),
            # rate limit
            rate_limit=True, 
            follow_camera=True,
            ):
        """
        by default visualize robot motion.
        also support visualize human motion by providing human_motion_data, to compare with robot motion.
        
        human_motion_data is a dict of {"human body name": (3d global translation, 3d global rotation)}.

        if rate_limit is True, the motion will be visualized at the same rate as the motion data.
        else, the motion will be visualized as fast as possible.
        """
        
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot # quat need to be scalar first! for mujoco
        self.data.qpos[7:] = dof_pos
        
        mj.mj_forward(self.model, self.data)
        
        if follow_camera and self._camera_follow:
            self.viewer.cam.lookat = self.data.xpos[self.model.body(self.robot_base).id]
            self.viewer.cam.distance = self.viewer_cam_distance
            self.viewer.cam.elevation = -10  # 正面视角，轻微向下看
            # self.viewer.cam.azimuth = 180    # 正面朝向机器人
        
        if human_motion_data is not None:
            # Clean custom geometry
            self.viewer.user_scn.ngeom = 0
            # Draw the task targets for reference
            for human_body_name, (pos, rot) in human_motion_data.items():
                draw_frame(
                    pos,
                    R.from_quat(rot, scalar_first=True).as_matrix(),
                    self.viewer,
                    human_point_scale,
                    pos_offset=human_pos_offset,
                    joint_name=human_body_name if show_human_body_name else None
                    )

        for flag_type, flag_id, flag_val in self._pending_flag_restores:
            if flag_type == "vis":
                self.viewer.opt.flags[flag_id] = flag_val
            elif flag_type == "rnd":
                self.viewer.opt.flags[flag_id] = flag_val
        self._pending_flag_restores.clear()

        self.viewer.sync()
        if rate_limit is True:
            self.rate_limiter.sleep()

        if self.record_video:
            # Use renderer for proper offscreen rendering
            self.renderer.update_scene(self.data, camera=self.viewer.cam)
            img = self.renderer.render()
            self.mp4_writer.append_data(img)
    
    def close(self):
        self.viewer.close()
        time.sleep(0.5)
        if self.record_video:
            self.mp4_writer.close()
            print(f"Video saved to {self.video_path}")
