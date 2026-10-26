# Sorting task session summary (2026-10-26 / 27)

Unitree G1 EDU, Dex3 hands, right-arm table sorting (coffee cup → Spot A, domol bottle → Spot B).
Code: `g1/academy/` on the robot's Jetson. Nothing is committed to git.

Backups taken before changes:

- `backups/sort_v2_20261026_232454/`: V1 and V2 files, before the cup-collision fixes
- `backups/sort_v2_20261026_235436/`: before the park sequence and the stuck-transport fixes
- `backups/sort_v2_20261027_002944/`: before the stale-finger, set-down and smooth-rise work

All changes were checked in the simulator (`sim_sort_v2.py`, harness in `tests/test_sort_v2.py`). The latest changes have not been run on the robot yet. The full pytest suite was not re-run.

---

## 1. Starting point: V1 (`g1_dashboard.py`, `sorting.py`, `grasp_pipeline.py`)

| Problem | Steps taken |
|---|---|
| Hand closed around the bottle, then opened without lifting; joints crackled (several controllers commanding the arm) | A single controller takes over the arm. Grasp height measured from the object top. Pre-grasp pose. |
| The hand pushed the other object | Objects other than the target are treated as obstacles (cylinders). The sorting view overlays the forward-kinematics end-effector (EE) position and the planned trajectory. |
| Palm not level / rubbing the table | Level-palm grasp with a minimum hand height above the table. |
| Detection boxes didn't line up with the objects | OpenAI vision (gpt-4.1) labels the depth-based object proposals (`openai_vision.py`), selected with `--vision-backend`. |
| UI | EF dark theme and logo as default, "Copy log" button. Default objects cup/bottle, maximum joint step and speed. |

## 2. V2 rewrite (`g1_dashboard_v2.py`, `sorting_v2.py`, `grasp_pipeline_v2.py`, `sort_v2_config.py`, `sorting_state_machine.py`, `reach_map_v2.py`, `sim_sort_v2.py`)

- An explicit state machine with phases and failure-specific recovery.
- One arm controller publishing to `rt/arm_sdk`, with gravity-sag compensation.
- A motion validator covering the whole-hand table geometry, obstacles and the carried object.
- Tiered level-palm IK and full 6D IK, with a guard against sudden joint jumps.
- Planning checks every key pose and the path between them before any motion.
- Tactile and kinematic grip evidence. Calibration is loaded from `sort_v2_calibration.json`.
- Simulator and test harness.

## 3. From "not working / unreachable" to working

| Problem | Cause | Steps taken |
|---|---|---|
| Sort refused: "Spot A/B out of the arm's reach"; later the motion stopped after the hand opened | The drop spots were on the robot's left. Spot A's centre was 0.51 m from the right shoulder, but the palm reaches at most 0.4318 m (measured over the joint space). The planner aimed at the spot centre. | Quick reach check of the drop-off route that fails early. Clear messages. **Drop at the nearest reachable point on the spot**. The operator moved the patterns to the right. |
| Unclear why points were unreachable | — | **Reach view** (`reach_map_v2.py`): a map of reachable table cells coloured by reason, computed in 4 processes and cached in `.reach_cache/`. |
| Was the camera calibration wrong? | Checked | The camera position (x 47.65 mm, z 462.68 mm from the waist yaw joint, 48° down) was confirmed correct; no change needed. |
| No ground truth for the planner's assumptions | — | `ik_calibration.py`: records a remote-controlled pick and place and analyses it (where the cup sat in the hand, camera offset, tilt, lift). A demo matched V2's grasp model. |
| "Carried object bottom too close to the table" for every finger direction | The release height was measured from the marker plane, which sits 1–1.5 cm below the depth table plane used by the safety check | The release height now uses the higher of the two planes. This failure no longer repeats across all finger directions. |
| thumb_1 jammed when opening/closing | The thumb caught on the curled fingers | Opening moves the fingers first; closing moves the thumb first. |
| Fingers stuck on the thigh at the start | — | The arm first swings out from the body (shoulder roll), then rises. |

After these fixes the first real runs worked, with remaining bugs.

## 4. Fixes after the first working runs

### 4.1 EE landing on top of the cup
- **Cause:** the fingertips sit 13 cm past the palm. With the cup close to the body, the torso keep-out limited the pre-grasp pull-back to about 5 cm. The fingertips then came down over the cup, and the target was not checked as an obstacle.
- **Steps:**
  - A thin-finger check keeps the fingers clear of the target during the clearance and pre-grasp moves.
  - The pull-back is sized from what is actually needed.
  - Finger directions up to ±60° are tried.
  - Measured-pose checks get 1 cm sideways slack, so a plan with only 0.2 mm to spare isn't aborted mid-move.
- **Later replaced by a side approach** (4.5).

### 4.2 Pressing "sort" with the cup already at the palm
- **Symptom:** "move away from the body: path rejected: wrist z below …".
- **Steps:**
  - A resume path: the object is detected between the fingers, the hand closes, and the sort goes straight to the drop-off.
  - The move-away step runs only when the hand is really beside the body.

### 4.3 Arm release at the end hit the table
- **Steps:** the start-up sequence is recorded (hanging pose → swung out → after the rise → recipe poses). At the end it is played back in reverse, and the arms are released only after that.
  - The final roll back in is best effort, because the hanging pose sits inside the workspace check's torso margin.
  - If the park fails, the arms stay engaged and a warning is logged.

### 4.4 Transport stopped 21 mm short, then recovery failed
- **Cause:** the sag compensation saturated (0.15 rad). Recovery then planned a 0 mm "up" move.
- **Steps:**
  - Raised the sag cap to 0.20 rad (at most 12 Nm per joint).
  - Transport, release, retreat and recovery moves count as reached within 3 cm (`short_move_accept_m`).
  - Recovery rises from the measured pose.

### 4.5 Approach still collided with the object → side approach
- **Steps:**
  - The pre-grasp now sits beside the object, on the side away from the palm (the robot's right for the right hand). The object edge stays 2 cm (`side_approach_margin_m`) clear of the hand body.
  - The hand comes down vertically there, then slides sideways along the palm normal onto the grasp.
  - Planning also checks the torso keep-out along whole paths.
  - Positions rejected because another object was in the way are retried after each placement.

### 4.6 Stale fingers
- **Finding:** the Dex3 reported no fault. Motor status was 0, hand error 0, temperatures 42–50 °C. So staleness is detected from motion: a joint commanded at least 0.3 rad that moved less than 0.05 rad.
- **Steps:**
  - `sdk_wrapper.py` snapshot now includes motor temperatures, status codes, modes and the hand error.
  - A recovery ladder runs on every open and close (prep, drop spot, recovery, final close, manual buttons):
    1. re-engage and retry in the other order;
    2. go limp, re-engage, retry slowly;
    3. wiggle the joint;
    4. fail, naming the joint and its motor health.
  - It covers thumb_1/2, middle_0/1 and index_0/1.
  - **Finger check before every grasp** in free space: close 35 % and reopen, every joint verified. Motors at 70 °C or more cool down limp first. If the check can't recover a finger, the sort stops at the prep pose with an error.
  - During the grasp, joints that never moved don't count as grip evidence (a stale thumb could look like a grip). A resulting jam leads to open, recover, close again.
  - At the drop spot: if the fingers won't open fully but the camera sees the object on the spot, the sort continues.
  - A failed open at the prep pose was retried with a staged order.

### 4.7 Gentle placing
- **Steps:** below the release pose, the hand lowers at 1 cm/s towards where the object's bottom meets the table, at most 1.5 cm further. It stops when the descent stalls, compared against how far the command actually moved. It then holds the measured pose so it doesn't keep pressing, and opens. The retreat rises above the object's top.
- **Settings:** `place_touchdown`, `touchdown_speed_m_s`, `touchdown_overshoot_m`.

### 4.8 Stuttering rise beside the body (and the reverse)
- **Cause:** IK was solved every control step without a fixed hand orientation. The ticks became uneven (62 poses planned for 6 s took 9 s).
- **Steps:** the vertical rise is planned before moving (1 cm steps on one IK branch) and played back at a constant joint speed (`prep_rise_speed_rad_s`). The park descent replays it in reverse. In simulation the path deviates at most 1.6 mm from vertical.

### 4.9 After four good runs: lid collision on lifting, index finger not closing, stuttering manual close
- **Lid:** after opening on the spot, the palm first moves horizontally away from the object along the palm normal, the reverse of the side approach. The distance is at least `release_slide_m` (5 cm) or the approach's side offset, shortened to what the arm can reach while keeping the finger direction. Only then does it lift. The same applies after a recovery set-down.
- **Index not closing on the bottle:** the grip held without it, so the pick went ahead. Now a finger that never moved during the close triggers open (with finger recovery) and close once more, while the object still rests on the table.
- **Stuttering manual close:**
  - Two manual hand commands could run at the same time, because the buttons were only refused during a sort. With finger recovery, an open can take several seconds.
  - All hand command sequences now run one at a time (`Dex3Grip.cmd_lock`), and a second button press is refused with "previous hand command still running".
  - Visible staged retries in the log ("recovering", "recovered after …") are the recovery ladder at work.
- Backup: `backups/sort_v2_20261027_012655/`.

## 5. Open points / next checks on the robot
- Confirm the palm side assumption of the side approach: the hand should move toward the object palm first.
- Check that the set-down detects resistance on the real arm. If it releases early or presses too long, tune `touchdown_stall_ratio` / `touchdown_window_s`.
- The cause of the stale finger is still unknown. The new log lines (which joint, which recovery step helped, motor health) should narrow it down.
- Camera tilt and depth scale: there is a 1–1.5 cm gap between the marker plane and the depth table plane. More `ik_calibration.py` carries in different hand directions would calibrate it.
- `hand_pose_navigation/run_hand_pose_nav.py` still uses camera z = 681.79 mm (not used by the sort).
- Re-run the full test suite: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest academy/tests -q -p no:cacheprovider` from `g1/`.
