// Offline unit tests for include/teleop_safety.hpp (no robot, no CUDA, no DDS).
// Build & run:  ./tests/run_offline_tests.sh
#include <vector>
#include <cstdio>
#include <cstdlib>

#include "teleop_safety.hpp"

using namespace teleop_safety;

static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond, msg)                                                         \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      std::printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);             \
    }                                                                            \
  } while (0)

static JointArray DefaultIsaac() {
  JointArray q{};
  for (int i = 0; i < kNumJoints; ++i) { q[i] = default_angles[HwIndexOfIsaac(i)]; }
  return q;
}

static int IsaacOfHw(int hw) {
  for (int i = 0; i < kNumJoints; ++i) {
    if (HwIndexOfIsaac(i) == hw) { return i; }
  }
  return -1;
}

static bool Throws(const std::function<void()>& f) {
  try {
    f();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}

// Drives a Supervisor tick by tick and checks per-tick invariants.
struct Harness {
  Config cfg;
  Supervisor sup;
  double t = 0.0;
  int64_t live = 0;
  JointArray prev_pos{};
  JointArray prev_vel{};
  bool have_prev = false;
  double max_dpos_excess = 0.0;   // worst |Δpos| beyond vmax·dt
  double max_dvel_excess = 0.0;   // worst |Δvel| beyond amax·dt
  double max_limit_excess = 0.0;  // worst position beyond ref limits
  bool nonfinite = false;

  explicit Harness(const Config& c) : cfg(c), sup(c) {}

  void Tick(const JointArray& raw, bool new_frame, bool deadman, double temp = 40.0, Quat quat = {1, 0, 0, 0},
            bool streamed = true, uint64_t source = 1, int64_t live_step = 1, bool heading_reinit = false) {
    auto& in = sup.input();
    in.now_s = t;
    in.streamed = streamed;
    in.source_id = source;
    in.new_frame = new_frame;
    in.raw_pos.assign(3, raw);  // a short window: current + 2 future frames
    in.raw_quat.assign(3, quat);
    in.measured_q = DefaultIsaac();
    in.max_motor_temp_c = temp;
    in.deadman_age_s = deadman ? 0.05 : 10.0;
    in.heading_reinit = heading_reinit;
    if (new_frame) { live += live_step; }
    in.live_frame_index = live;
    sup.Update();
    const auto& out = sup.output();
    const auto& lim = sup.limits();
    const double dt = cfg.ControlDt();
    for (int i = 0; i < kNumJoints; ++i) {
      double p = out.pos[0][i], v = out.vel[0][i];
      if (!std::isfinite(p) || !std::isfinite(v)) { nonfinite = true; }
      max_limit_excess = std::max({max_limit_excess, lim.lo[i] - p, p - lim.hi[i]});
      if (have_prev) {
        max_dpos_excess = std::max(max_dpos_excess, std::abs(p - prev_pos[i]) - lim.vmax[i] * dt);
        max_dvel_excess = std::max(max_dvel_excess, std::abs(v - prev_vel[i]) - lim.amax[i] * dt);
      }
      prev_pos[i] = p;
      prev_vel[i] = v;
    }
    have_prev = true;
    t += dt;
  }
  const JointArray& Ref() const { return sup.output().pos[0]; }
};

static void TestConfig() {
  std::printf("[test] configuration loading and validation\n");
  Config def;
  CHECK(!Throws([&] { ConfigLoader::Validate(def); }), "built-in defaults validate");

  const char* path = "/tmp/claude_teleop_safety_test.conf";
  {
    std::ofstream f(path);
    f << "# comment\nsmoothing_time_ms = 180   # inline\nmax_velocity_fraction=0.2\ncsv_log_path = /tmp/x.csv\n";
  }
  Config c = ConfigLoader::LoadFile(path);
  CHECK(c.smoothing_time_ms == 180.0 && c.max_velocity_fraction == 0.2, "file values parsed");
  CHECK(c.csv_log_path == "/tmp/x.csv", "string value parsed");
  CHECK(c.mocap_timeout_ms == 200.0, "absent key keeps conservative default");

  CHECK(Throws([&] { Config x; ConfigLoader::ApplyOverride(x, "max_velocty_fraction=0.2"); }), "unknown key rejected");
  CHECK(Throws([&] { Config x; ConfigLoader::ApplyOverride(x, "mocap_timeout_ms=abc"); }), "non-numeric rejected");
  CHECK(Throws([&] { Config x; ConfigLoader::ApplyOverride(x, "mocap_timeout_ms=200ms"); }), "trailing junk rejected");
  CHECK(Throws([&] { Config x; x.temperature_stop_c = 95; ConfigLoader::Validate(x); }), "temperature stop above 90 C rejected");
  CHECK(Throws([&] { Config x; x.temperature_reduce_c = 86; ConfigLoader::Validate(x); }), "reduce >= stop rejected");
  CHECK(Throws([&] { Config x; x.control_rate_hz = 100; ConfigLoader::Validate(x); }), "control rate other than 50 Hz rejected");
  CHECK(Throws([&] { Config x; x.init_ramp_s = 1.0; ConfigLoader::Validate(x); }), "init ramp faster than original 3 s rejected");
  CHECK(Throws([&] { Config x; x.max_velocity_fraction = 0.0; ConfigLoader::Validate(x); }), "zero velocity fraction rejected");
  Config hot;
  hot.max_velocity_fraction = 0.6;
  CHECK(!ConfigLoader::Advisories(hot).empty(), "advisory for aggressive velocity fraction");
  std::remove(path);
}

static void TestFilterStep() {
  std::printf("[test] filter respects velocity/acceleration limits on a 1.5 rad step\n");
  Config cfg;
  JointFilterState s;
  JointArray start{};
  s.Reset(start);
  StepLimits lim;
  for (int i = 0; i < kNumJoints; ++i) {
    lim.vmax[i] = 2.0;
    lim.amax[i] = 12.0;
    lim.lo[i] = -3;
    lim.hi[i] = 3;
  }
  JointArray target{};
  target.fill(1.5);
  double dt = cfg.ControlDt(), worst_dp = 0, worst_dv = 0, overshoot = 0;
  JointArray prev_p = s.pos, prev_v = s.vel;
  for (int k = 0; k < 400; ++k) {
    StepFilter(s, target, dt, 0.2, lim);
    worst_dp = std::max(worst_dp, std::abs(s.pos[0] - prev_p[0]));
    worst_dv = std::max(worst_dv, std::abs(s.vel[0] - prev_v[0]));
    overshoot = std::max(overshoot, s.pos[0] - 1.5);
    prev_p = s.pos;
    prev_v = s.vel;
  }
  CHECK(worst_dp <= 2.0 * dt + 1e-9, "per-cycle position change within vmax*dt");
  CHECK(worst_dv <= 12.0 * dt + 1e-9, "per-cycle velocity change within amax*dt");
  CHECK(std::abs(s.pos[0] - 1.5) < 1e-3, "converges to target");
  CHECK(overshoot < 0.01, "no significant overshoot");
  std::printf("    worst dq/cycle=%.4f rad (limit %.4f), worst dv/cycle=%.4f (limit %.4f), overshoot=%.5f\n", worst_dp,
              2.0 * dt, worst_dv, 12.0 * dt, overshoot);
}

static void TestStartupHoldRampFollow() {
  std::printf("[test] startup: hold without dead-man, ramp from measured pose, then follow\n");
  Config cfg;
  Harness h(cfg);
  JointArray raw = DefaultIsaac();
  int ls = IsaacOfHw(15);  // left_shoulder_pitch
  raw[ls] = -1.0;          // operator far from the robot's pose

  for (int k = 0; k < 50; ++k) h.Tick(raw, true, /*deadman=*/false);
  CHECK(h.sup.mode() == Mode::kHold, "HOLD while dead-man not held");
  CHECK(std::abs(h.Ref()[ls] - default_angles[15]) < 1e-6, "reference stays at measured pose without dead-man");

  int ticks_to_follow = -1;
  for (int k = 0; k < 600; ++k) {
    h.Tick(raw, true, true);
    if (ticks_to_follow < 0 && h.sup.mode() == Mode::kFollow) ticks_to_follow = k + 1;
  }
  double ramp_s = ticks_to_follow * cfg.ControlDt();
  CHECK(ticks_to_follow > 0, "reaches FOLLOW");
  CHECK(std::abs(ramp_s - cfg.startup_ramp_s) < 0.05, "ramp lasts startup_ramp_s");
  CHECK(std::abs(h.Ref()[ls] - (-1.0)) < 1e-3, "reaches operator pose");
  CHECK(h.max_dpos_excess <= 1e-9, "no per-cycle step above vmax*dt");
  CHECK(h.max_dvel_excess <= 1e-9, "no per-cycle accel above amax*dt");
  CHECK(!h.nonfinite, "outputs finite");
  std::printf("    ramp %.2f s\n", ramp_s);
}

static void TestInvalidFrames() {
  std::printf("[test] NaN / far-out-of-limit frames rejected; near-limit clamped\n");
  Config cfg;
  Harness h(cfg);
  JointArray raw = DefaultIsaac();
  for (int k = 0; k < 300; ++k) h.Tick(raw, true, true);
  JointArray before = h.Ref();

  JointArray nan_frame = raw;
  nan_frame[IsaacOfHw(18)] = std::numeric_limits<double>::quiet_NaN();
  for (int k = 0; k < 5; ++k) h.Tick(nan_frame, true, true);
  CHECK(h.sup.stats().invalid_frames == 5, "5 NaN frames counted invalid");
  CHECK(!h.nonfinite, "NaN never reaches reference");

  JointArray garbage = raw;
  garbage[IsaacOfHw(3)] = 5.0;  // knee 5 rad
  for (int k = 0; k < 3; ++k) h.Tick(garbage, true, true);
  CHECK(h.sup.stats().invalid_frames == 8, "far-out-of-limit frames rejected");
  CHECK(std::abs(h.Ref()[IsaacOfHw(3)] - before[IsaacOfHw(3)]) < 1e-6, "knee not driven by garbage frame");

  JointArray near = raw;
  int el = IsaacOfHw(18);  // left elbow, upper 2.0944
  near[el] = 2.2;          // beyond limit but within tolerance
  for (int k = 0; k < 300; ++k) h.Tick(near, true, true);
  CHECK(h.Ref()[el] <= 2.0944 - cfg.reference_limit_margin_rad + 1e-9, "near-limit target clamped inside margin");
  CHECK(h.max_limit_excess <= 1e-9, "reference never outside limits");
}

static void TestTimeoutAndDeadman() {
  std::printf("[test] mocap timeout and dead-man release smoothly stop and hold\n");
  Config cfg;
  Harness h(cfg);
  JointArray base = DefaultIsaac();
  int ls = IsaacOfHw(15);
  // Follow a moving target so the reference has velocity when the stream stops.
  int k = 0;
  for (; k < 400; ++k) {
    JointArray raw = base;
    raw[ls] = 0.2 + 0.8 * std::sin(0.02 * k);
    h.Tick(raw, true, true);
  }
  CHECK(h.sup.mode() == Mode::kFollow, "following before dropout");
  JointArray last = base;
  last[ls] = 0.2 + 0.8 * std::sin(0.02 * (k - 1));
  // Stream stops: same object, no new frames.
  int ticks_to_hold = -1;
  for (int n = 0; n < 50; ++n) {
    h.Tick(last, false, true);
    if (ticks_to_hold < 0 && h.sup.mode() == Mode::kHold) ticks_to_hold = n + 1;
  }
  double t_hold = ticks_to_hold * cfg.ControlDt();
  CHECK(ticks_to_hold > 0, "enters HOLD on mocap timeout");
  CHECK(t_hold > cfg.mocap_timeout_ms / 1000.0 - 0.021 && t_hold < cfg.mocap_timeout_ms / 1000.0 + 0.041,
        "HOLD triggered at ~mocap_timeout_ms");
  CHECK(std::abs(h.sup.output().vel[0][ls]) < 1e-6, "velocity decays to zero while holding");
  CHECK(h.sup.stats().stale_events == 1, "one stale event logged");
  double held = h.Ref()[ls];
  for (int n = 0; n < 50; ++n) h.Tick(last, false, true);
  CHECK(std::abs(h.Ref()[ls] - held) < 1e-9, "position held during outage");
  std::printf("    hold after %.0f ms\n", t_hold * 1000.0);

  // Stream resumes with a far target: must ramp, not jump.
  JointArray far = base;
  far[ls] = -1.2;
  h.Tick(far, true, true);
  CHECK(h.sup.mode() == Mode::kRamp, "resumption goes through RAMP");
  for (int n = 0; n < 100; ++n) h.Tick(far, true, true);
  // Dead-man released mid-motion.
  h.Tick(far, true, false);
  CHECK(h.sup.mode() == Mode::kHold, "dead-man release -> HOLD");
  for (int n = 0; n < 100; ++n) h.Tick(far, true, false);
  CHECK(std::abs(h.sup.output().vel[0][ls]) < 1e-6, "stopped after dead-man release");
  CHECK(h.max_dpos_excess <= 1e-9 && h.max_dvel_excess <= 1e-9, "limits respected through stop/resume");
}

static void TestTemperature() {
  std::printf("[test] temperature reduce / stop with hysteresis\n");
  Config cfg;
  Harness h(cfg);
  JointArray raw = DefaultIsaac();
  for (int k = 0; k < 300; ++k) h.Tick(raw, true, true, 40.0);
  double vmax_normal = h.sup.limits().vmax[0];
  h.Tick(raw, true, true, 72.0);
  CHECK(h.sup.temp_state() == TempState::kReduced, "72 C -> REDUCED");
  CHECK(std::abs(h.sup.limits().vmax[0] - vmax_normal * cfg.temperature_reduce_factor) < 1e-12, "limits scaled");
  h.Tick(raw, true, true, 86.0);
  CHECK(h.sup.temp_state() == TempState::kStopped && h.sup.mode() == Mode::kHold, "86 C -> STOPPED + HOLD");
  h.Tick(raw, true, true, 82.0);
  CHECK(h.sup.temp_state() == TempState::kStopped, "hysteresis keeps STOPPED at 82 C");
  h.Tick(raw, true, true, 79.0);
  CHECK(h.sup.temp_state() == TempState::kReduced && h.sup.mode() == Mode::kRamp, "79 C -> REDUCED and ramps back in");
  h.Tick(raw, true, true, 66.0);
  CHECK(h.sup.temp_state() == TempState::kReduced, "hysteresis keeps REDUCED at 66 C");
  h.Tick(raw, true, true, 64.0);
  CHECK(h.sup.temp_state() == TempState::kNormal, "64 C -> normal");
}

static void TestDroppedFramesAndWindow() {
  std::printf("[test] dropped-frame accounting and window rollout\n");
  Config cfg;
  Harness h(cfg);
  JointArray raw = DefaultIsaac();
  for (int k = 0; k < 10; ++k) h.Tick(raw, true, true);
  h.Tick(raw, true, true, 40.0, {1, 0, 0, 0}, true, 1, /*live_step=*/6);
  CHECK(h.sup.stats().dropped_frames == 5, "index gap of 6 counts 5 dropped frames");
  for (int k = 0; k < 300; ++k) h.Tick(raw, true, true);
  CHECK(h.sup.output().pos.size() == 3, "window of 3 frames rolled out");
  bool continuous = true;
  for (size_t f = 1; f < h.sup.output().pos.size(); ++f) {
    for (int i = 0; i < kNumJoints; ++i) {
      if (std::abs(h.sup.output().pos[f][i] - h.sup.output().pos[f - 1][i]) > h.sup.limits().vmax[i] * cfg.ControlDt() + 1e-9) continuous = false;
    }
  }
  CHECK(continuous, "future window frames obey the same limits");
}

static void TestHeadingAdopt() {
  std::printf("[test] heading recapture adopts operator yaw without turning the robot\n");
  Config cfg;
  Harness h(cfg);
  JointArray raw = DefaultIsaac();
  for (int k = 0; k < 10; ++k) h.Tick(raw, true, true);
  Quat yaw90 = {std::cos(M_PI / 4), 0, 0, std::sin(M_PI / 4)};
  h.Tick(raw, true, true, 40.0, yaw90, true, 1, 1, /*heading_reinit=*/true);
  CHECK(QuatAngle(h.sup.output().quat[0], yaw90) < 1e-6, "yaw adopted immediately on heading recapture");
  Quat yaw180 = {0, 0, 0, 1};
  h.Tick(raw, true, true, 40.0, yaw180);
  double step = QuatAngle(h.sup.output().quat[0], yaw90);
  CHECK(step <= cfg.max_root_angular_velocity * cfg.ControlDt() + 1e-9, "subsequent yaw change rate limited");
}

static void TestOutputGuard() {
  std::printf("[test] motor-target guard\n");
  std::array<float, kNumJoints> prev{}, q{};
  for (int i = 0; i < kNumJoints; ++i) prev[i] = q[i] = static_cast<float>(default_angles[i]);
  q[5] = 0.5f;  // left ankle roll beyond 0.2618 and a 0.5 rad jump
  auto st = GuardMotorTargets(q, prev, true, 0.35);
  CHECK(st.jump_limited == 1, "jump limited");
  CHECK(q[5] <= 0.2618f + 1e-6f, "clamped to URDF limit");
  q = prev;
  q[18] = 3.0f;  // elbow above 2.0944 without jump guard
  st = GuardMotorTargets(q, prev, false, 0.35);
  CHECK(st.clamped == 1 && q[18] <= 2.0944f + 1e-6f, "clamped without prev");
  q = prev;
  q[3] = std::numeric_limits<float>::quiet_NaN();
  st = GuardMotorTargets(q, prev, true, 0.35);
  CHECK(st.nonfinite, "NaN detected");
}

int main() {
  TestConfig();
  TestFilterStep();
  TestStartupHoldRampFollow();
  TestInvalidFrames();
  TestTimeoutAndDeadman();
  TestTemperature();
  TestDroppedFramesAndWindow();
  TestHeadingAdopt();
  TestOutputGuard();
  std::printf("\n%d checks, %d failures -> %s\n", g_checks, g_failures, g_failures ? "FAIL" : "PASS");
  return g_failures ? 1 : 0;
}
