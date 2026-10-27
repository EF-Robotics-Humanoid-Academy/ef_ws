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

### 4.10 Cup set-down: fingers did not open, arm moved back and forth
- **Chain of events (log 16:16):**
  1. The cup was gripped at the lowest allowed height. During the set-down its bottom reached the table, but the hand slid down along the cup, so the descent never stalled and no resistance was detected. The measured wrist ended about 3 cm below the expected contact.
  2. The carried-object check computed the cup bottom from the grasp-time hang, got "z=-0.057 m" (inside the table), and aborted.
  3. The recovery tried to carry the cup back; its "up" move was refused below the normal wrist floor. ERROR, with the hand still closed.
  4. In ERROR the reference stayed about 1.5 cm above the real hand, so the sag compensation kept pushing against the hand wedged on the cup and table: the back-and-forth motion.
- **Steps:**
  - The set-down also stops when the *measured* wrist reaches its floor, or when the grip load changes (a held finger shifting ≥ 0.06 rad, or a contact sensor changing clearly).
  - In set-down mode the carried-object-bottom check is off; the hand's own table check stays.
  - Failure during a set-down: open at the spot (the object stands on the table), move away, rise above the object, and count the placement.
  - The recovery rise may start from below the normal wrist floor.
  - ERROR holds the arm at its measured pose.
- Backup: `backups/sort_v2_20261027_162158/`.

### 4.11 Hand stayed down after placing the cup; bottle closing failed twice
- **Hand stayed down (log 16:36):**
  - **Cause:** while carrying the cup the sag compensation was saturated (0.200 rad), with the hand about 2 cm below its command. At the set-down stop, the command was moved to the measured pose (the 4.10 change). That lowered it by the gap, and the hand sank onto the table (wrist 0.059 → 0.029 m). Every later move, including upward ones, was refused, because the measured pose was already below its floor → ERROR.
  - **Steps:**
    - The stop holds the hand where it is: the measured pose plus the command-to-hand gap measured while moving freely, with the integral frozen (`ArmController.integrate = False`). Every new move re-enables the integral.
    - ERROR freezes the integral and no longer moves the command.
    - A move that rises out of a too-low pose may run as long as the hand does not go lower than where it started (`LOW_POSE_MARKERS`).
- **Bottle closing failed twice:**
  - **Cause:** the bottle touched the fingers as soon as the palm arrived. Fingers that "did not move at all" were treated as stale motors (4.6) and not counted as grip evidence → jam, twice.
  - **Step:** a liveness probe. Each finger that did not move is commanded 0.12 rad back toward open. A live motor follows, so the finger counts as touching the object; a stale one does not.
- Backup: `backups/sort_v2_20261027_164021/`.

### 4.12 Bottle pushed before closing; cup set-down pressed into the table
- **Bottle pushed:** the hand only closed within 20 mm of the planned grasp, so a bottle sitting closer than detected was pushed. Now the touch baseline is recorded at the pre-grasp. Within 4 cm of the grasp (palm level, finger direction ok), the hand closes as soon as any touch sensor feels the object (`approach_contact_window_m`). `close()` keeps that baseline (up to 15 s old) instead of re-recording it while already touching.
- **Cup, transport 48 mm short (17:03):** Spot A's centre was at y = +0.070, at the edge of the right arm's reach; blocked, not sagging. Recovery returned the cup correctly. Keep spots at y ≤ 0.
- **Cup, set-down pressed into the table (17:06):**
  - **Cause:** IK stalled the command (fixed finger direction), so the stall test, which needs a descending command, never fired. Meanwhile the sag integral wound up to its 0.200 cap and pressed the hand into the cup and table (tracking error 0.45 rad). The ERROR state then froze that wound-up integral, so every later move, including the manual retract, failed with "arm not following".
  - **Steps:**
    - `ArmController.mark_free()` / `relax()`: relax restores the last free-arm state (integral, and command = measured + free gap), so the arm holds where it is without pressing.
    - The set-down marks free first, runs with the integral frozen (`move_linear(integrate=False)`), lets the finger direction turn (`yaw_free`), and also detects contact when the hand stays 12 mm (`touchdown_push_m`) above its command beyond its steady free-descent lag.
    - It relaxes at the end, even after a failure.
    - Recovery and ERROR relax when tracking error is above 0.08 rad (`PRESSING_TRACKING_ERROR_RAD`), otherwise they freeze the integral.
- **Tested in simulation:** two objects, cup close to the robot, resume, stale finger, set-down with resistance; no false contact. Written but not yet run (shell permission check was unavailable): `scratchpad/stall.py`, the blocked set-down with an IK stall.
- Backup: `backups/sort_v2_20261027_170916/`.

### 4.13 Descent to the pre-grasp crawled and stopped short (cup far ahead)
- **Log 17:26:**
  - The cup was at x = 0.345 and Spot A at y = +0.113, with the table at z ≈ −0.025 m.
  - Planning took 66 s, because two finger directions failed on the drop-off at the left.
  - The chosen +15° plan put the side-approach pre-grasp at (0.331, −0.126, 0.074), about 0.40 m from the shoulder at a low height, near the 0.43 m maximum.
  - There the IK solution wandered by tenths of a radian per 3 mm step. Each step waited for the command to catch up (lag ≤ 0.02 rad), so the descent crawled (steps 37→42 took 10 s) and failed with "no progress, blocked at step 42/45".
- **Steps:**
  - Planning solves the descent to the pre-grasp and the approach in 5 mm steps with the execution IK, and rejects a finger direction whose joints would move more than 0.15 rad/cm (`path_max_joint_rate_rad_per_m`; normal ≈ 0.03 rad/cm). In simulation with the robot's table height, every direction for that cup was rejected (0.11–0.14 rad per 5 mm), with a message to move the object.
  - Set-down: when the command itself no longer descends (reach edge), it ends with a release there instead of a "no progress" failure.
- **Layout guidance at this table height:** cup about x 0.27–0.32, y −0.05 to −0.15; spots x ≤ 0.36, y ≤ 0. A slightly higher table enlarges the reachable area.
- Backup: `backups/sort_v2_20261027_172946/`.

### 4.14 Same layout (cup (0.344, 0.001), Spot A y = +0.113): every finger direction rejected
- The new check (4.13) rejected all approaches correctly, and 0° / −15° failed on the drop-off at the left.
- **Tried in simulation and reverted:**
  - a "compact side approach" second pass: the joints also jump on the final approach to the grasp itself, so the grasp point, not the approach start, is at the reach edge;
  - gripping 3 cm higher: the route passed, but the cup centre then sat 7.4 cm below the wrist, outside the 6 cm capture range.
- **Layout probe at the robot's table height (−0.025 m):** Spot A at y = +0.113 is never feasible (drop-off), even with the cup at x = 0.31. Cup (0.29, −0.05) with Spot A (0.36, +0.02) works.
- No code change kept; backup of that attempt in `backups/sort_v2_20261027_174445/`.

### 4.15 Long planning and refusal although the cup is reachable
- **Log 17:53:**
  - Planning took 60 s, then every direction was refused. The arm can reach the cup: at 17:26 it got to step 42/45.
  - **Causes:**
    1. Execution solved the IK fresh at every 3 mm step. Near the stretched arm the solutions drifted, and the descent crawled to a stop. The 4.13 limit then refused those routes outright.
    2. Every pose had to keep the finger direction within 8°, including the transport and the drop-off. Those checks failed for Spot A on the left (9.5–13°) and cost about 20 s each on the Jetson.
- **Steps:**
  - **Planned paths are executed:** the descent to the pre-grasp and the approach are solved in 5 mm steps during planning, stored in `plan.data["paths"]` and followed by `move_linear(q_path=…)`, interpolated to the move's speed and validated per step, with no per-tick IK. The joint-rate limit was relaxed to 0.4 rad/cm (`path_max_joint_rate_rad_per_m` = 40), so only real configuration changes are refused.
  - **Finger direction per phase:** carrying and drop-off are checked and lowered with a level palm and free finger direction (`yaw_free`). The clearance and pre-grasp poses use 12° (`path_yaw_tolerance_rad`). The final approach keeps 8°.
  - **Set-down:** "no progress" during the set-down ends it with a release there (reach edge).
- **Simulation, robot table height, cup (0.344, 0.001), Spot A (0.344, +0.113):** planned on the first direction in about 4.5 s (previously refused after a minute); cup picked and placed on Spot A. Regression set and stall scenario pass.
- Backup: `backups/sort_v2_20261027_175730/`.

### 4.16 Lower table: grip height offset
- The table used now is lower than yesterday's desk (z ≈ −0.025 m in the robot frame).
- **New `grip_height_offset_m`** (dashboard tuning, −1.5 … +3 cm, default −1 cm): shifts the grasp height relative to the hand-model floor. The effective hand-to-table clearance at the grasp is 1.5 cm plus the offset (never below 0), via `SortV2Config.grasp_hand_clearance_m()`. The set-down's expected contact follows automatically (measured hang); its lowest point lets the hand model reach the table surface (`place_hand_clearance_m()` = 0).
- Simulation on the low table: grasp at z = 0.065 m (1 cm lower).

### 4.17 Waist assist (torso yaw) for out-of-reach pick / drop-off spots
- **Log 18:23:** Spot A at y = +0.093. While carrying, the arm sagged 1.8 cm below its transport height, the cup came too close to the other object ("carried object would hit obstacle"), and the recovery's "up" move was refused → ERROR.
  - Fixes: transport 2 cm higher above obstacles (`transport_extra_clearance_m` 0.05); a rising move may leave a "carried object would hit obstacle" pose (`LOW_POSE_MARKERS`).
- **Waist assist:**
  - `ArmController.set_waist_yaw()` ramps joint 12 at `waist_yaw_speed_rad_s` (0.2) with the engage waist gains (kp 480, kd 10) in the same `rt/arm_sdk` message as the arm, capped at `max_waist_yaw_rad` 0.45.
  - `GraspPipelineV2.turn_waist()` turns and waits for the joint to arrive.
  - When planning fails for reach (UNREACHABLE / IK failure), or the drop-off plan cannot lower the object onto the spot (`_set_down_shortfall`), the sorter turns the torso at the preparation pose, hand empty, to `waist_assist_yaws_deg` (15°, then 25°; left for the right arm, right for targets far to the right).
  - It then invalidates the view (`view_epoch`, settle 1.5 s), re-detects and plans again. The camera rides on the torso, so the whole planner works unchanged in the turned torso frame.
  - After each object (placed or recovered) and before the park at the end, the waist turns back to 0°.
  - Dashboard toggle: `waist_assist`.
- Retreat after release at the reach edge: falls back to an orientation-free rise when the level palm has no IK. A release followed by a failure is now checked and counted as a placement.
- **Simulation (waist joint, camera and objects in the turned torso frame):** the 18:23 layout turns to +15° and sets the cup down 9 mm above its floor (previously 5 cm). The 17:53 layout turns to +15° then +25° and sets the cup down fully, 1 cm from Spot A's centre. Regression unchanged.
- Backups: `backups/sort_v2_20261027_182918/`, `backups/sort_v2_20261027_183619/` (sim).

## 5. Open points / next checks on the robot
- Confirm the palm side assumption of the side approach: the hand should move toward the object palm first.
- Check that the set-down detects resistance on the real arm. If it releases early or presses too long, tune `touchdown_stall_ratio` / `touchdown_window_s`.
- The cause of the stale finger is still unknown. The new log lines (which joint, which recovery step helped, motor health) should narrow it down.
- Camera tilt and depth scale: there is a 1–1.5 cm gap between the marker plane and the depth table plane. More `ik_calibration.py` carries in different hand directions would calibrate it.
- `hand_pose_navigation/run_hand_pose_nav.py` still uses camera z = 681.79 mm (not used by the sort).
- Re-run the full test suite: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest academy/tests -q -p no:cacheprovider` from `g1/`.
