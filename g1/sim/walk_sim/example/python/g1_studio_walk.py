#!/usr/bin/env python3
"""G1 free-standing walk in MuJoCo with the Surface Studio walking controller.

A Python port of Surface Studio's live walker (``~/3d_models/surface/brain.js``) and of its physics-side
balance loop and joint motors (``~/3d_models/surface/physics.js``: ``balance()``, ``control()``, ``measure()``).
No DDS and no elastic band: the robot stands on the floor and must balance by itself.

Velocity command (vx m/s, wz deg/s) → footsteps (step length vx·T, turn wz·T, T = Tds + Tss) → centre-of-
pressure plan on the feet → centre of mass from the divergent component of motion (DCM ξ = x + ẋ/ω, inverted
pendulum, ω² = g/h) → pelvis placed so the whole robot's CoM is on the plan → both legs by damped least-squares
IK (ankle point + foot flat) → per-joint q target and dq feed-forward.

Closed loop, every control tick (125 Hz), from the simulated robot:
  * walker: measured CoM / CoM speed / feet. Each step re-anchors to the real feet; a fast capture-point
    deviation (> 1.5 cm beyond a slowly learned bias) re-aims the swing foot (capture point), or, when standing,
    triggers a recovery step.
  * balance: CoM error against the plan over the supporting feet → weight shift (hip ±, ankle ∓, feet stay
    flat, kp 1.5, kd 0.5, ≤ 6 cm) and upper body upright (hip pitch, 0.8 deg/deg).
  * motors: tau = kp·(q* − q) + kd·(dq* − dq), kp = min(800, 10·effort), kd = min(40, 0.5·effort), capped at the
    joint's effort limit (the MJCF actuator range). The damping term is integrated implicitly by MuJoCo
    (implicitfast + per-dof damping), like Rapier's implicit joint motor in the studio.

Differences from the studio: the MJCF's masses/limits (ankle 50 Nm, not 35), MuJoCo contacts instead of Rapier,
the torque cap is enforced every physics step (the studio checks it at the control rate), and the arm swing is
contralateral (the studio's shoulder-pitch swing has the same-side sign for the G1's joint convention).

Usage:
  python3 g1_studio_walk.py                    # headless scripted demo (walk, top speed, curve, turn, back, push)
  python3 g1_studio_walk.py --viewer           # same demo in the MuJoCo viewer
  python3 g1_studio_walk.py --viewer --keys    # steer: ↑ ↓ speed, ← → turn, 0 stop, P push sideways (100 N)
  python3 g1_studio_walk.py --log run.csv      # per-tick log: command, pelvis, CoM, leg q*/q/tau
"""

import argparse
import csv
import math
from pathlib import Path
import time

import mujoco
import numpy as np

G = 9.81
D2R = math.pi / 180.0
MOTOR_COUNT = 29
LEG = {"L": list(range(0, 6)), "R": list(range(6, 12))}   # hip pitch, roll, yaw, knee, ankle pitch, roll
HP, HR, KNEE, AP, AR = 0, 1, 3, 4, 5                      # offsets inside a leg
SHOULDER_PITCH = {"L": 15, "R": 22}
ANKLE_BODY = {"L": "left_ankle_roll_link", "R": "right_ankle_roll_link"}
OTHER = {"L": "R", "R": "L"}

WALK_DEFAULT = dict(ground=0.0, Tss=0.55, Tds=0.2, Tshift=0.6, lift=0.05, crouch=0.05, maxStep=0.18,
                    maxBack=0.08, maxTurn=15.0, fx=0.03, capture=1.0)
BALANCE_GAINS = dict(kp=1.5, kd=0.5, kt=0.8, ap=1.0, ar=1.0, hp=1.0, hr=0.0)
MOTOR_K, MOTOR_D = 800.0, 40.0
CMD_LIMITS = dict(vx=(-0.1, 0.2), wz=(-30.0, 30.0))       # the studio's arrow-key limits


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smooth(u):
    return u * u * (3.0 - 2.0 * u)


def fwd(yaw):
    return np.array([math.cos(yaw), math.sin(yaw)])


def left(yaw):
    return np.array([-math.sin(yaw), math.cos(yaw)])


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotvec(R):
    """Rotation vector (axis × angle) of a rotation matrix."""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.reshape(9))
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def yaw_of(xmat):
    R = xmat.reshape(3, 3)
    return math.atan2(R[1, 0], R[0, 0])


def foot(p, yaw):
    return {"p": np.asarray(p, dtype=float).copy(), "yaw": float(yaw)}


def copy_foot(f):
    return foot(f["p"], f["yaw"])


class RobotIndex:
    """Motor → joint addresses, limits and effort, read from the MJCF actuators."""

    def __init__(self, model):
        self.qadr, self.dadr, self.lo, self.hi, self.effort = [], [], [], [], []
        for i in range(MOTOR_COUNT):
            j = int(model.actuator_trnid[i, 0])
            self.qadr.append(int(model.jnt_qposadr[j]))
            self.dadr.append(int(model.jnt_dofadr[j]))
            self.lo.append(float(model.jnt_range[j, 0]))
            self.hi.append(float(model.jnt_range[j, 1]))
            self.effort.append(float(model.actuator_ctrlrange[i, 1]))
        self.qadr, self.dadr = np.array(self.qadr), np.array(self.dadr)
        self.lo, self.hi, self.effort = np.array(self.lo), np.array(self.hi), np.array(self.effort)
        self.ankle = {sd: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ANKLE_BODY[sd]) for sd in "LR"}
        self.pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")


class Walker:
    """brain.js createWalker(): command → footsteps → ZMP → DCM → CoM → pelvis → leg IK (kinematic model)."""

    def __init__(self, model, idx, q_start, pelvis_start, opt=None):
        self.P = dict(WALK_DEFAULT, **(opt or {}))
        self.m, self.idx = model, idx
        self.d = mujoco.MjData(model)
        self.q = np.asarray(q_start, dtype=float).copy()
        self.z_base = float(pelvis_start[2])
        self.jp, self.jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        self._set_pelvis(pelvis_start[0], pelvis_start[1], self.z_base, 0.0)
        self._fk()
        self.feet, self.y0 = {}, {}
        for sd in "LR":
            b = idx.ankle[sd]
            self.feet[sd] = foot(self.d.xpos[b][:2], yaw_of(self.d.xmat[b]))
            self.y0[sd] = float(self.d.xpos[b][2])
        self.width = float(np.linalg.norm(self.feet["L"]["p"] - self.feet["R"]["p"]))
        c0 = self.d.subtree_com[idx.pelvis].copy()
        self.h = c0[2] - self.P["crouch"] - min(self.y0.values()) + 0.035
        self.w = math.sqrt(G / self.h)
        self.sp0 = {sd: self.q[SHOULDER_PITCH[sd]] for sd in "LR"}
        self.seed = np.zeros(6)
        self.seed[KNEE] = 20 * D2R                                   # G1 knees bend positive
        self.cmd = {"vx": 0.0, "wz": 0.0}
        self.bias, self.dev, self.recoveries = None, np.zeros(2), 0
        self.mode, self.step, self.stand_t, self.stand_from, self.next_swing = "stand", None, 0.0, None, "L"
        self.com, self.px, self.py = c0[:2].copy(), float(pelvis_start[0]), float(pelvis_start[1])
        self.t0, self.xi_ref, self.last_t = None, c0[:2].copy(), 0.0
        self.pelvis_pose = (self.px, self.py, self.z_base, 0.0)

    # ── kinematic model ──
    def _set_pelvis(self, x, y, z, yaw):
        self.d.qpos[0:3] = (x, y, z)
        self.d.qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    def _fk(self):
        self.d.qpos[self.idx.qadr] = self.q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)

    def ik(self, sd, T, yaw):
        """Ankle point to T, foot flat at heading yaw: damped least squares, ≤ 10° per joint per iteration."""
        legs = LEG[sd]
        b, Rt = self.idx.ankle[sd], rot_z(yaw)
        if abs(self.q[legs[KNEE]]) < 3 * D2R:
            self.q[legs] = np.clip(self.q[legs] + self.seed, self.idx.lo[legs], self.idx.hi[legs])
        cols = self.idx.dadr[legs]
        for _ in range(12):
            self._fk()
            e = np.concatenate((T - self.d.xpos[b], rotvec(Rt @ self.d.xmat[b].reshape(3, 3).T)))
            if np.linalg.norm(e) < 2e-6:
                break
            mujoco.mj_jacBody(self.m, self.d, self.jp, self.jr, b)
            J = np.vstack((self.jp[:, cols], self.jr[:, cols]))
            y = np.linalg.solve(J @ J.T + 1e-8 * np.eye(6), e)
            dv = np.clip(J.T @ y, -10 * D2R, 10 * D2R)
            self.q[legs] = np.clip(self.q[legs] + dv, self.idx.lo[legs], self.idx.hi[legs])

    def com_of_pose(self):
        self._fk()
        return self.d.subtree_com[self.idx.pelvis].copy()

    # ── gait ──
    def foot_z(self, f, sd):                    # where the foot carries weight: its middle, a little inside
        inn = -0.01 if sd == "L" else 0.01
        return f["p"] + fwd(f["yaw"]) * self.P["fx"] + left(f["yaw"]) * inn

    def mid(self, fs):
        return (self.foot_z(fs["L"], "L") + self.foot_z(fs["R"], "R")) / 2

    def moving(self):
        return abs(self.cmd["vx"]) > 0.02 or abs(self.cmd["wz"]) > 2

    def place(self, stance, sd, closing):       # nominal footstep for swing side sd, from the standing foot
        P = self.P
        Ts = P["Tss"] + P["Tds"]
        turn = 0.0 if closing else clamp(self.cmd["wz"] * Ts, -P["maxTurn"], P["maxTurn"]) * D2R
        yaw, ym = stance["yaw"] + turn, stance["yaw"] + turn / 2
        L = 0.0 if closing else clamp(self.cmd["vx"] * Ts, -P["maxBack"], P["maxStep"])
        s = 1.0 if sd == "L" else -1.0
        return foot(stance["p"] + fwd(ym) * L + left(ym) * self.width * s, yaw)

    def _new_step(self, sd, t, closing, to, **kw):
        P = self.P
        st = dict(sd=sd, t0=t, tLift=t + P["Tds"], tLand=t + P["Tds"] + P["Tss"], closing=closing, to=to,
                  first=False, recovery=False, lifted=False, nominal=None)
        st.update(kw)
        return st

    def zmp_plan(self, t):                      # ZMP segments over the next ~2.6 s
        seg, H = [], t + 2.6
        fs = {"L": copy_foot(self.feet["L"]), "R": copy_foot(self.feet["R"])}
        if self.mode == "stand":
            if self.stand_from is not None and t < self.stand_t + 0.5:
                seg.append((self.stand_t, self.stand_t + 0.5, self.stand_from, self.mid(fs)))
            seg.append((max(t, self.stand_t + 0.5), H, self.mid(fs), self.mid(fs)))
            return seg
        s, tt = self.step, t
        sw, closing = s["sd"], s["closing"]
        for _ in range(5):
            if tt >= H:
                break
            st = OTHER[sw]
            frm = self.mid(fs) if s["first"] else self.foot_z(fs[sw], sw)
            to = self.foot_z(fs[st], st)
            seg.append((s["t0"], s["tLift"], frm, to))
            seg.append((s["tLift"], s["tLand"], to, to))
            fs = dict(fs)
            fs[sw] = s["nominal"] or s["to"]
            tt = s["tLand"]
            if closing:
                m = self.mid(fs)
                seg.append((tt, tt + 0.5, self.foot_z(fs[sw], sw), m))
                seg.append((tt + 0.5, H, m, m))
                return seg
            nsw, cl = st, not self.moving()
            s = self._new_step(nsw, tt, cl, self.place(fs[sw], nsw, cl))
            sw, closing = nsw, cl
        last = seg[-1]
        seg.append((last[1], max(H, last[1] + 0.1), last[3], last[3]))
        return seg

    def dcm_now(self, t):                       # ξ(t): integrate ξ̇ = ω(ξ − p) backwards from the horizon's end
        seg = self.zmp_plan(t)
        dt, end = 0.01, seg[-1][1]
        n = int(math.ceil((end - t) / dt))
        times = t + dt * np.arange(n + 1)
        times[-1] = end
        p = np.zeros((n + 1, 2))
        for i, tt in enumerate(times):
            q = next((x for x in seg if tt < x[1]), seg[-1])
            u = smooth(clamp((tt - q[0]) / (q[1] - q[0]), 0.0, 1.0)) if q[1] > q[0] else 1.0
            p[i] = q[2] + (q[3] - q[2]) * u
        xi, k = p[n].copy(), math.exp(-self.w * dt)
        for i in range(n - 1, -1, -1):
            xi = p[i] + (xi - p[i]) * k
        return xi

    def update(self, t, meas):
        """meas: com (2), vel (2), feet {L, R: {p, yaw}} from the simulated robot. Returns the 29 joint targets."""
        P, w = self.P, self.w
        if self.t0 is None:
            self.t0 = t
        first = t == self.t0
        dtc = clamp(t - self.last_t, 0.0, 0.05)
        self.last_t = t
        # measured capture point against the plan's: a slow, steady offset is learned (1.5 s); only a fast
        # deviation beyond 1.5 cm — a push, a slip — moves the feet
        if meas is not None:
            xi_m = meas["com"] + meas["vel"] / w
            e = xi_m - self.xi_ref
            if self.bias is None:
                self.bias = e.copy()
            big = np.linalg.norm(e - self.bias) > 0.04
            k = 0.0 if big else dtc / 1.5
            self.bias = self.bias + (e - self.bias) * k
            dz = lambda x: 0.0 if abs(x) < 0.015 else x - math.copysign(0.015, x)
            self.dev = np.array([dz(v) for v in e - self.bias])
        # gait events
        if self.mode == "stand" and self.moving() and t - self.stand_t > 0.3:
            sw = self.next_swing
            self.step = self._new_step(sw, t, False, self.place(self.feet[OTHER[sw]], sw, False), first=True,
                                       tLift=t + P["Tshift"], tLand=t + P["Tshift"] + P["Tss"])
            self.mode = "walk"
        # pushed while standing: when the capture point leaves the feet, a quick step towards it
        if self.mode == "stand" and meas is not None and P["capture"] and t - self.t0 > 1:
            hd = (self.feet["L"]["yaw"] + self.feet["R"]["yaw"]) / 2
            df, dl = float(self.dev @ fwd(hd)), float(self.dev @ left(hd))
            if df > 0.1 or df < -0.08 or abs(dl) > 0.04:
                sw = ("L" if dl > 0 else "R") if abs(dl) > 0.03 else self.next_swing
                self.step = self._new_step(sw, t, False, self.place(self.feet[OTHER[sw]], sw, True), first=True,
                                           recovery=True, tLift=t + 0.1, tLand=t + 0.48)
                self.mode = "walk"
                self.recoveries += 1
        st_step = self.step
        if self.mode == "walk" and st_step and not st_step["lifted"] and t >= st_step["tLift"]:
            st_step["lifted"] = True                       # lift-off: re-anchor to the real feet, aim the step
            st = OTHER[st_step["sd"]]
            if meas is not None:
                d = meas["feet"][st]["p"] - self.feet[st]["p"]
                if np.linalg.norm(d) < 0.2:
                    self.feet[st] = copy_foot(meas["feet"][st])
                    self.feet[st_step["sd"]] = copy_foot(meas["feet"][st_step["sd"]])
                    self.com = self.com + d
                    self.px += d[0]
                    self.py += d[1]
            st_step["from"] = copy_foot(self.feet[st_step["sd"]])
            st_step["nominal"] = self.place(self.feet[st], st_step["sd"], st_step["closing"] or st_step["recovery"])
            st_step["off"] = np.zeros(2)
            st_step["to"] = copy_foot(st_step["nominal"])
        # capture point: aim the swing foot where the body will need it at touch-down (first 70 % of the swing)
        if (self.mode == "walk" and st_step and st_step["lifted"] and meas is not None and P["capture"]
                and t < st_step["tLift"] + 0.7 * (st_step["tLand"] - st_step["tLift"])):
            st = OTHER[st_step["sd"]]
            g = math.exp(w * max(0.0, st_step["tLand"] - t))
            lim = 0.25 if st_step["recovery"] else 0.15
            want = np.clip(self.dev * g * P["capture"], -lim, lim)
            st_step["off"] = st_step["off"] + (want - st_step["off"]) * 0.3
            to = foot(st_step["nominal"]["p"] + st_step["off"], st_step["nominal"]["yaw"])
            lf, sg = left(self.feet[st]["yaw"]), (1.0 if st_step["sd"] == "L" else -1.0)
            side = float((to["p"] - self.feet[st]["p"]) @ lf)
            if side * sg < 0.16:                           # feet never cross
                to["p"] = to["p"] + lf * (0.16 - side * sg) * sg
            st_step["to"] = to
        if self.mode == "walk" and st_step and t >= st_step["tLand"]:   # touch-down
            self.feet[st_step["sd"]] = copy_foot(st_step["to"])
            st, nsw = st_step["sd"], OTHER[st_step["sd"]]
            if st_step["recovery"] and not self.moving():
                self.step = self._new_step(nsw, t, True, self.place(self.feet[st], nsw, True))
            elif st_step["closing"]:
                self.mode, self.stand_t = "stand", t
                self.stand_from, self.next_swing, self.step = self.foot_z(self.feet[st], st), nsw, None
            else:
                cl = not self.moving()
                self.step = self._new_step(nsw, t, cl, self.place(self.feet[st], nsw, cl))
        # centre of mass follows the DCM (ẋ = ω (ξ − x)), stable by construction
        self.xi_ref = self.dcm_now(t)
        self.com = self.com + w * (self.xi_ref - self.com) * dtc
        # feet targets (the swing foot on a smooth arc), pelvis height and heading
        T, yaw = {}, {}
        for sd in "LR":
            T[sd] = np.array([*self.feet[sd]["p"], self.y0[sd] + P["ground"]])
            yaw[sd] = self.feet[sd]["yaw"]
        s = self.step
        if self.mode == "walk" and s and s["lifted"] and t < s["tLand"]:
            u = clamp((t - s["tLift"]) / (s["tLand"] - s["tLift"]), 0.0, 1.0)
            sx, hu = (1 - math.cos(math.pi * u)) / 2, math.sin(math.pi * u)
            a, b = s["from"], s["to"]
            T[s["sd"]] = np.array([*(a["p"] + (b["p"] - a["p"]) * sx), self.y0[s["sd"]] + P["ground"] + P["lift"] * hu])
            yaw[s["sd"]] = a["yaw"] + (b["yaw"] - a["yaw"]) * sx
        dip = P["crouch"] * smooth(clamp((t - self.t0) / 0.8, 0.0, 1.0))
        heading = (yaw["L"] + yaw["R"]) / 2
        z = self.z_base - dip
        dx = float((T["R"][:2] - T["L"][:2]) @ fwd(heading))
        swing = clamp(dx * 60, -14, 14) * D2R               # contralateral: the arm opposite the forward foot comes forward
        for sd, sg in (("L", -1.0), ("R", 1.0)):
            i = SHOULDER_PITCH[sd]
            self.q[i] = clamp(self.sp0[sd] + sg * swing, self.idx.lo[i], self.idx.hi[i])
        for _ in range(5 if first else 1):                  # pelvis: wherever puts the whole robot's CoM on the plan
            self._set_pelvis(self.px, self.py, z, heading)
            self.ik("L", T["L"], yaw["L"])
            self.ik("R", T["R"], yaw["R"])
            c = self.com_of_pose()
            self.pelvis_pose = (self.px, self.py, z, heading)
            self.px += self.com[0] - c[0]
            self.py += self.com[1] - c[1]
        self.plan_com = c
        self.plan_feet = {sd: T[sd].copy() for sd in "LR"}
        self.plan_heading = heading
        return self.q.copy()


class StudioG1:
    """physics.js robot control on MuJoCo: walker targets + balance → torque-limited PD joint motors."""

    def __init__(self, model_path, dt=0.001, control_hz=125.0, walk_opt=None, balance=True):
        m = mujoco.MjModel.from_xml_path(str(model_path))
        m.opt.timestep = dt
        m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.m, self.d = m, mujoco.MjData(m)
        self.idx = idx = RobotIndex(m)
        self.kp = np.minimum(MOTOR_K, 10 * idx.effort)
        self.kd = np.minimum(MOTOR_D, 0.5 * idx.effort)
        m.dof_damping[idx.dadr] += self.kd          # the motor's damping, integrated implicitly by MuJoCo
        m.actuator_ctrllimited[:] = 0               # ctrl = cap(tau) + kd·dq: the net torque stays ≤ effort
        mujoco.mj_forward(m, self.d)
        self.q0 = self.d.qpos[idx.qadr].copy()
        self.walker = Walker(m, idx, self.q0, self.d.qpos[0:3].copy(), walk_opt)
        self.balance_on = balance
        self.n_ctrl = max(1, int(round(1.0 / (control_hz * dt))))
        self.t_ctrl = self.n_ctrl * dt
        self.rest_z = {sd: float(self.d.xpos[idx.ankle[sd]][2]) for sd in "LR"}
        self.root_z0 = float(self.d.qpos[2])
        self.goal, self.vt = self.q0.copy(), np.zeros(MOTOR_COUNT)
        self.prev_pose, self.prev_plan_com = None, None
        self.tau = np.zeros(MOTOR_COUNT)
        self.saturated, self.fallen, self.steps, self.bal = 0, None, 0, None
        self.push = None
        self._weight_shift_levers()

    def _weight_shift_levers(self):
        """physics.js robot.legs: tilting a standing leg (hip +δ, ankle −δ, foot stays flat) moves the pelvis over
        the foot by r metres per radian forward (pitch pair) / to the left (roll pair) — measured on this rig."""
        kin = mujoco.MjData(self.m)
        kin.qpos[:] = self.d.qpos
        delta, self.levers = 0.01, {}
        for sd in "LR":
            legs, b = LEG[sd], self.idx.ankle[sd]

            def ankle_after(h, a):
                q = kin.qpos.copy()
                kin.qpos[self.idx.qadr[legs[h]]] += delta
                kin.qpos[self.idx.qadr[legs[a]]] -= delta
                mujoco.mj_kinematics(self.m, kin)
                p = kin.xpos[b].copy()
                kin.qpos[:] = q
                return p
            mujoco.mj_kinematics(self.m, kin)
            p0 = kin.xpos[b].copy()
            rF = (p0 - ankle_after(HP, AP))[0] / delta
            rL = (p0 - ankle_after(HR, AR))[1] / delta
            self.levers[sd] = (rF, rL)

    # ── sensors (what physics.js measure() reads: the simulator's exact state) ──
    def measure(self):
        m, d, idx = self.m, self.d, self.idx
        mujoco.mj_subtreeVel(m, d)
        feet = {sd: foot(d.xpos[idx.ankle[sd]][:2], yaw_of(d.xmat[idx.ankle[sd]])) for sd in "LR"}
        return {"com": d.subtree_com[idx.pelvis][:2].copy(), "vel": d.subtree_linvel[idx.pelvis][:2].copy(),
                "com3": d.subtree_com[idx.pelvis].copy(), "vel3": d.subtree_linvel[idx.pelvis].copy(), "feet": feet}

    def balance(self, meas):
        """physics.js balance(): CoM where the plan has it over the supporting feet (ankle strategy, weight shift),
        at the plan's speed, and the upper body at the plan's tilt (hip strategy)."""
        d, idx, w, B = self.d, self.idx, self.walker, BALANCE_GAINS
        on = [sd for sd in "LR" if d.xpos[idx.ankle[sd]][2] - self.rest_z[sd] < 0.025]
        if not on:
            return None
        low = min(w.plan_feet[sd][2] for sd in "LR")
        plan_sup = [sd for sd in "LR" if w.plan_feet[sd][2] - low < 0.02]
        use = [sd for sd in plan_sup if sd in on] or on
        S = np.mean([d.xpos[idx.ankle[sd]][:2] for sd in use], axis=0)
        Sp = np.mean([w.plan_feet[sd][:2] for sd in use], axis=0)
        R = d.xmat[idx.pelvis].reshape(3, 3)
        f = np.array([R[0, 0], R[1, 0]])
        f /= np.linalg.norm(f)
        l = np.array([-f[1], f[0]])
        hf, hl = fwd(w.plan_heading), left(w.plan_heading)
        c, v = meas["com3"], meas["vel3"]
        pfw, plt = float((w.plan_com[:2] - Sp) @ hf), float((w.plan_com[:2] - Sp) @ hl)
        ef, el = float((c[:2] - S) @ f) - pfw, float((c[:2] - S) @ l) - plt
        pv = np.zeros(2) if self.prev_plan_com is None else (w.plan_com[:2] - self.prev_plan_com[:2]) / self.t_ctrl
        if np.linalg.norm(pv) > 3:
            pv = np.zeros(2)
        vf, vl = float((v[:2] - pv) @ f), float((v[:2] - pv) @ l)
        up = R[:, 2]
        asd = lambda x: math.degrees(math.asin(clamp(x, -1.0, 1.0)))
        pitch, roll = asd(up[:2] @ f), asd(up[:2] @ l)      # the plan's pelvis is upright
        return {"feet": set(on), "ef": ef, "el": el,
                "shF": clamp(-B["ap"] * (B["kp"] * ef + B["kd"] * vf), -0.06, 0.06),
                "shL": clamp(-B["ar"] * (B["kp"] * el + B["kd"] * vl), -0.06, 0.06),
                "hipP": clamp(B["hp"] * B["kt"] * pitch, -20, 20) * D2R, "hipR": clamp(B["hr"] * B["kt"] * roll, -20, 20) * D2R}

    def control(self):
        """physics.js control(): q* = walker pose + balance offsets (clamped to the joint range), dq* = the plan's
        own joint speed (feed-forward)."""
        meas = self.measure()
        pose = self.walker.update(self.d.time, meas)
        bal = self.balance(meas) if self.balance_on else None
        self.bal = bal
        goal = pose.copy()
        if bal:
            for sd in "LR":
                if sd not in bal["feet"]:
                    continue
                legs, (rF, rL) = LEG[sd], self.levers[sd]
                aF = bal["shF"] / rF if abs(rF) > 1e-5 else 0.0
                aL = bal["shL"] / rL if abs(rL) > 1e-5 else 0.0
                goal[legs[HP]] += aF + bal["hipP"]
                goal[legs[AP]] -= aF
                goal[legs[HR]] += aL + bal["hipR"]
                goal[legs[AR]] -= aL
        self.goal = np.clip(goal, self.idx.lo, self.idx.hi)
        vt = np.zeros(MOTOR_COUNT) if self.prev_pose is None else (pose - self.prev_pose) / self.t_ctrl
        vt[np.abs(vt) > 25] = 0.0
        self.vt, self.prev_pose = vt, pose
        self.prev_plan_com = self.walker.plan_com.copy()

    def motors(self):
        """tau = kp(q* − q) + kd(dq* − dq), capped at the effort limit. The −kd·dq part is MuJoCo's implicit joint
        damping, so ctrl carries the rest."""
        idx, d = self.idx, self.d
        q, dq = d.qpos[idx.qadr], d.qvel[idx.dadr]
        tau = self.kp * (self.goal - q) + self.kd * (self.vt - dq)
        sat = np.abs(tau) > idx.effort
        self.tau = np.clip(tau, -idx.effort, idx.effort)
        self.saturated = int(sat.sum())
        d.ctrl[:] = self.tau + self.kd * dq

    def step(self):
        if self.steps % self.n_ctrl == 0:
            self.control()
        self.motors()
        d = self.d
        d.xfrc_applied[self.idx.pelvis] = 0.0
        if self.push and self.push[0] <= d.time < self.push[1]:
            d.xfrc_applied[self.idx.pelvis, 0:3] = self.push[2]
        mujoco.mj_step(self.m, d)
        self.steps += 1
        if self.fallen is None and d.qpos[2] < self.root_z0 - 0.35:
            self.fallen = d.time

    def command(self, vx=None, wz=None):
        if vx is not None:
            self.walker.cmd["vx"] = clamp(vx, *CMD_LIMITS["vx"])
        if wz is not None:
            self.walker.cmd["wz"] = clamp(wz, *CMD_LIMITS["wz"])

    def shove(self, force=60.0, duration=0.15):
        """Push the pelvis towards the robot's left (negative force: right) for `duration` seconds."""
        lf = left(yaw_of(self.d.xmat[self.idx.pelvis]))
        self.push = (self.d.time, self.d.time + duration, np.array([lf[0] * force, lf[1] * force, 0.0]))


# scripted demo, like the studio's "Physics: driven live" recording: (time s, vx m/s, wz deg/s, push N)
DEMO = [(0.0, 0.0, 0.0, None), (1.5, 0.1, 0.0, None), (6.0, 0.2, 0.0, None), (10.0, 0.15, 12.0, None),
        (14.0, 0.0, 20.0, None), (18.0, -0.08, 0.0, None), (22.0, 0.0, 0.0, None), (25.0, 0.0, 0.0, 100.0)]


def run(args):
    model_path = Path(__file__).resolve().parents[2] / "unitree_robots" / "g1" / "scene_29dof.xml"
    sim = StudioG1(model_path, dt=args.dt, control_hz=args.control_hz, balance=not args.no_balance)
    script = [] if args.keys else list(DEMO)
    writer, log_file = None, None
    if args.log:
        log_file = open(args.log, "w", newline="")
        writer = csv.writer(log_file)
        names = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
        writer.writerow(["t", "vx", "wz", "mode", "pelvis_x", "pelvis_y", "pelvis_z", "yaw_deg", "com_x", "com_y",
                         "bal_ef", "bal_el", "saturated"]
                        + [f"{s}_{n}_{k}" for s in ("l", "r") for n in names for k in ("q_des", "q", "tau")])

    def log():
        d, w, b = sim.d, sim.walker, sim.bal or {}
        row = [round(d.time, 4), w.cmd["vx"], w.cmd["wz"], w.mode, *np.round(d.qpos[0:3], 4),
               round(math.degrees(yaw_of(d.xmat[sim.idx.pelvis])), 2), *np.round(d.subtree_com[sim.idx.pelvis][:2], 4),
               round(b.get("ef", 0.0), 4), round(b.get("el", 0.0), 4), sim.saturated]
        for i in range(12):
            row += [round(float(sim.goal[i]), 4), round(float(d.qpos[sim.idx.qadr[i]]), 4), round(float(sim.tau[i]), 2)]
        writer.writerow(row)

    def advance_script():
        while script and sim.d.time >= script[0][0]:
            _, vx, wz, push = script.pop(0)
            sim.command(vx, wz)
            if push:
                sim.shove(push)
            print(f"t={sim.d.time:5.2f}s  command vx={vx:+.2f} m/s  wz={wz:+.0f} deg/s" + (f"  push {push:.0f} N" if push else ""))

    def key_callback(key):
        w = sim.walker.cmd
        if key == 265:
            sim.command(vx=round(w["vx"] + 0.05, 2))
        elif key == 264:
            sim.command(vx=round(w["vx"] - 0.05, 2))
        elif key == 263:
            sim.command(wz=w["wz"] + 10)
        elif key == 262:
            sim.command(wz=w["wz"] - 10)
        elif key == 48:
            sim.command(0.0, 0.0)
        elif key == 80:
            sim.shove(100.0)
        print(f"command vx={sim.walker.cmd['vx']:+.2f} m/s  wz={sim.walker.cmd['wz']:+.0f} deg/s")

    duration = args.duration if args.duration else (math.inf if args.keys else 36.0)
    max_sat, start = 0, time.monotonic()
    try:
        if args.viewer:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=key_callback if args.keys else None) as v:
                wall0, sim0 = time.monotonic(), sim.d.time
                while v.is_running() and sim.d.time < duration:
                    advance_script()
                    for _ in range(sim.n_ctrl):
                        sim.step()
                    if writer and sim.steps % sim.n_ctrl == 0:
                        log()
                    max_sat = max(max_sat, sim.saturated)
                    v.sync()
                    lag = (sim.d.time - sim0) - (time.monotonic() - wall0)
                    if lag > 0:
                        time.sleep(lag)
        else:
            next_report = 1.0
            while sim.d.time < duration and sim.fallen is None:
                advance_script()
                for _ in range(sim.n_ctrl):
                    sim.step()
                if writer:
                    log()
                max_sat = max(max_sat, sim.saturated)
                if sim.d.time >= next_report:
                    next_report += 1.0
                    w, b = sim.walker, sim.bal or {}
                    print(f"t={sim.d.time:5.2f}s  {w.mode:5s}  pelvis ({sim.d.qpos[0]:+.2f}, {sim.d.qpos[1]:+.2f}, "
                          f"{sim.d.qpos[2]:.3f}) yaw {math.degrees(yaw_of(sim.d.xmat[sim.idx.pelvis])):+6.1f}°  "
                          f"CoM err ({b.get('ef', 0) * 1000:+.0f}, {b.get('el', 0) * 1000:+.0f}) mm  "
                          f"motors at limit {sim.saturated}")
    finally:
        if log_file:
            log_file.close()
    d = sim.d
    print(f"\n{'FELL at %.2f s' % sim.fallen if sim.fallen else 'Stayed up'} · simulated {d.time:.1f} s in "
          f"{time.monotonic() - start:.1f} s · pelvis at ({d.qpos[0]:+.2f}, {d.qpos[1]:+.2f}) m, "
          f"yaw {math.degrees(yaw_of(d.xmat[sim.idx.pelvis])):+.0f}° · recovery steps {sim.walker.recoveries} · "
          f"most motors at their limit at once: {max_sat}")
    return 1 if sim.fallen else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--viewer", action="store_true", help="show the MuJoCo viewer (real time)")
    parser.add_argument("--keys", action="store_true", help="steer with the arrow keys instead of the scripted demo (needs --viewer)")
    parser.add_argument("--duration", type=float, help="simulated seconds (default: 36 for the demo)")
    parser.add_argument("--dt", type=float, default=0.001, help="physics step in seconds (default 0.001)")
    parser.add_argument("--control-hz", type=float, default=125.0, help="walker + balance rate (studio: 120 Hz)")
    parser.add_argument("--no-balance", action="store_true", help="turn off the weight-shift / upright balance loop")
    parser.add_argument("--log", help="write a per-control-tick CSV log here")
    args = parser.parse_args()
    if args.keys and not args.viewer:
        parser.error("--keys needs --viewer")
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
