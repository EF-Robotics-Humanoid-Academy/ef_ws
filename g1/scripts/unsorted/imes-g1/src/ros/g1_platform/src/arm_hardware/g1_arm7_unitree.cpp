#include "g1_arm_control_hardware/g1_arm7_unitree.h"

ArmUnitreeControlNode::ArmUnitreeControlNode()
    : weight_(0.0f),
      weight_rate_(0.2f),
      kp_(30.f), kd_(1.0f), dq_(0.f), tau_ff_(0.f),
      control_dt_(0.02f),
      max_joint_velocity_(0.3f),
      max_joint_delta_(max_joint_velocity_ * control_dt_),
      sleep_time_(std::chrono::milliseconds(static_cast<int>(control_dt_ / 0.001f)))
{
    std::cout << "Initializing G1 Dual Arm Control Node" << std::endl;

    // Unitree Robot setup for G1 (hg namespace on eth0)
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

    unitree_arm_sdk_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
    unitree_arm_sdk_publisher_->InitChannel();

    unitree_arm_sdk_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicArmSDKSub));
    unitree_arm_sdk_subscriber_->InitChannel(
        std::bind(&ArmUnitreeControlNode::g1_state, this, std::placeholders::_1), 1);
}

void ArmUnitreeControlNode::arms_move_unrestricted(const std::array<float, 14> &target_angles)
{
    // Start from last commanded (or zeros)
    std::array<float, 14> q{};
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        q = current_cmd_;
    }

    // Split to L/R and clamp to limits
    std::array<float, 7> curL{}, curR{}, tgtL{}, tgtR{};
    for (int j = 0; j < 7; ++j)
    {
        curL[j] = q[j];
        tgtL[j] = target_angles[j];
    }
    for (int j = 0; j < 7; ++j)
    {
        curR[j] = q[7 + j];
        tgtR[j] = target_angles[7 + j];
    }
    tgtL = clampArray7(tgtL);
    tgtR = clampArray7(tgtR);

    // Steps from per-joint velocity limits
    int stepsL = stepsFromVel(curL, tgtL);
    int stepsR = stepsFromVel(curR, tgtR);
    std::array<float, 7> desL = curL, desR = curR;

    while (stepsL > 0 || stepsR > 0)
    {
        if (stepsL > 0)
            stepToward7(desL, tgtL, stepsL);
        if (stepsR > 0)
            stepToward7(desR, tgtR, stepsR);
        {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            for (int j = 0; j < 7; ++j)
                current_cmd_[j] = desL[j];
            for (int j = 0; j < 7; ++j)
                current_cmd_[7 + j] = desR[j];
            // Use move gains during segment; fixed weight each tick
            send_cmd_14(current_cmd_, kp_move_, kd_move_, true, weight_fixed_);
        }
        std::this_thread::sleep_for(sleep_time_);
    }
    // Final hold at exact targets
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        for (int j = 0; j < 7; ++j)
            current_cmd_[j] = tgtL[j];
        for (int j = 0; j < 7; ++j)
            current_cmd_[7 + j] = tgtR[j];
    }
    send_cmd_14(current_cmd_, kp_hold_, kd_hold_, true, weight_fixed_);
}

void ArmUnitreeControlNode::arms_calibrate()
{
    std::cout << "[arms_calibrate] delay " << startup_delay_ << "s" << std::endl;
    if (startup_delay_ > 0.f)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(startup_delay_ * 1000.f)));

    if (!state_received_.load())
    {
        std::cout << "[arms_calibrate] waiting for state..." << std::endl;
        for (int i = 0; i < 50 && !state_received_.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Current measured L/R
    std::array<float, 7> curL{}, curR{};
    read_current_lr(curL, curR);
    auto tgtL = clampArray7(startup_left_pose_);
    auto tgtR = clampArray7(startup_right_pose_);

    // Only move if significant
    float max_diff_L = 0.f, max_diff_R = 0.f;
    for (int j = 0; j < 7; ++j)
    {
        max_diff_L = std::max(max_diff_L, std::fabs(curL[j] - tgtL[j]));
        max_diff_R = std::max(max_diff_R, std::fabs(curR[j] - tgtR[j]));
    }
    if (max_diff_L <= min_move_threshold_ && max_diff_R <= min_move_threshold_)
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        for (int j = 0; j < 7; ++j)
            current_cmd_[j] = curL[j];
        for (int j = 0; j < 7; ++j)
            current_cmd_[7 + j] = curR[j];
        send_cmd_14(current_cmd_, kp_hold_, kd_hold_, /*set_weight*/ true, weight_fixed_);
        std::cout << "[arms_calibrate] already near startup; holding." << std::endl;
        return;
    }

    // Build 14-DoF targets
    std::array<float, 14> q_tgt{};
    for (int j = 0; j < 7; ++j)
        q_tgt[j] = tgtL[j];
    for (int j = 0; j < 7; ++j)
        q_tgt[7 + j] = tgtR[j];

    // Start from measured (desired = measured)
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        for (int j = 0; j < 7; ++j)
            current_cmd_[j] = curL[j];
        for (int j = 0; j < 7; ++j)
            current_cmd_[7 + j] = curR[j];
    }

    // Compute steps from time if given, else from velocity limits
    int stepsL = (startup_time_ > 0.f) ? std::max(1, static_cast<int>(std::ceil(startup_time_ / control_dt_)))
                                       : stepsFromVel(curL, tgtL);
    int stepsR = (startup_time_ > 0.f) ? std::max(1, static_cast<int>(std::ceil(startup_time_ / control_dt_)))
                                       : stepsFromVel(curR, tgtR);
    std::array<float, 7> desL = curL, desR = curR;

    std::cout << "[arms_calibrate] moving to startup over ~" << control_dt_ * std::max(stepsL, stepsR)
              << "s (steps L=" << stepsL << ", R=" << stepsR << ")" << std::endl;

    while (stepsL > 0 || stepsR > 0)
    {
        if (stepsL > 0)
            stepToward7(desL, tgtL, stepsL);
        if (stepsR > 0)
            stepToward7(desR, tgtR, stepsR);
        {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            for (int j = 0; j < 7; ++j)
                current_cmd_[j] = desL[j];
            for (int j = 0; j < 7; ++j)
                current_cmd_[7 + j] = desR[j];
            // Use transition gains while moving; fixed weight each tick
            send_cmd_14(current_cmd_, kp_transition_, kd_transition_, true, weight_fixed_);
        }
        std::this_thread::sleep_for(sleep_time_);
    }

    // Hold at target
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        current_cmd_ = q_tgt;
    }
    send_cmd_14(current_cmd_, kp_hold_, kd_hold_, true, weight_fixed_);
    std::cout << "[arms_calibrate] complete; holding." << std::endl;
}

void ArmUnitreeControlNode::arms_stop()
{
    std::cout << "[arms_stop] gradual stop..." << std::endl;

    // Measure (fallback: last command)
    std::array<float, 7> curL{}, curR{};
    std::array<float, 14> measured{};
    if (state_received_.load())
    {
        read_current_lr(curL, curR);
        for (int j = 0; j < 7; ++j)
            measured[j] = curL[j];
        for (int j = 0; j < 7; ++j)
            measured[7 + j] = curR[j];
    }
    else
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        measured = current_cmd_;
    }

    // Start from last commanded
    std::array<float, 14> q{};
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        q = current_cmd_;
    }

    // Steps limited by stop_time_
    int steps = std::max(1, static_cast<int>(std::ceil(stop_time_ / control_dt_)));
    while (steps > 0)
    {
        float decel = 1.0f;
        if (steps <= decel_steps_)
        {
            float prog = static_cast<float>(steps) / static_cast<float>(decel_steps_);
            decel = 0.3f + 0.7f * prog;
        }
        for (int j = 0; j < 14; ++j)
        {
            // per-joint limit (use left limits for [0..6], right for [7..13])
            const int k = (j < 7) ? j : (j - 7);
            const float per_step = vel_lim_[k] * control_dt_ * decel;
            const float delta = clampf(measured[j] - q[j], -per_step, per_step);
            q[j] += delta;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            current_cmd_ = q;
            // Transition gains during stopping
            send_cmd_14(current_cmd_, kp_transition_, kd_transition_, true, weight_fixed_);
        }
        std::this_thread::sleep_for(sleep_time_);
        --steps;
    }
    // Hold final
    send_cmd_14(current_cmd_, kp_hold_, kd_hold_, true, weight_fixed_);
    std::cout << "[arms_stop] stop complete; holding." << std::endl;
}

void ArmUnitreeControlNode::g1_state(const void *message)
{
    auto state = static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
    std::lock_guard<std::mutex> lk(state_mtx_);
    memcpy(&unitree_msg_state_, state, sizeof(unitree_msg_state_));

    const int n = state->motor_state().size();

    for (int j = 0; j < 7; ++j)
    {
        int idL = idx_left[j];
        if (idL >= 0 && idL < n)
            tracking_pos_[j] = state->motor_state().at(idL).q();
    }
    for (int j = 0; j < 7; ++j)
    {
        int idR = idx_right[j];
        if (idR >= 0 && idR < n)
            tracking_pos_[7 + j] = state->motor_state().at(idR).q();
    }

    state_received_.store(true);
}

void ArmUnitreeControlNode::shutdown()
{
    arms_stop();
}