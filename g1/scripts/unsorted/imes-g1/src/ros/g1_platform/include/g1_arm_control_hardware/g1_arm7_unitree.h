#ifndef G1_ARM7_UNITREE_H_
#define G1_ARM7_UNITREE_H_

/*
@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>
#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <cstring>
#include <atomic>
#include <algorithm>
#include <cmath>

// Unitree DDS for G1 (using hg namespace, not go2)
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;
static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicArmSDKSub = "rt/lowstate";

// G1 Joint indices (matching your working file)
enum JointIndex
{
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,
    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,
    // Trunk
    kWaistYaw,
    kWaistRoll,
    kWaistPitch,
    // Left arm (7 DoF)
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWristRoll,
    kLeftWristPitch,
    kLeftWristYaw,
    // Right arm (7 DoF)
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWristRoll,
    kRightWristPitch,
    kRightWristYaw,
    // Misc / reserved
    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

class ArmUnitreeControlNode
{
public:
    ArmUnitreeControlNode();
    void shutdown();
    void arms_stop();
    void arms_calibrate();
    void arms_move_unrestricted(const std::array<float, 14> &target_angles); // Now 14 DoF (7+7)
    void g1_state(const void *message);

    // Updated for 7+7 DoF (left arm + right arm)
    std::array<float, 14> tracking_pos_{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,   // left arm 7 DoF
                                         0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}; // right arm 7 DoF

protected:
    // Unitree for G1 (using hg namespace)
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> unitree_arm_sdk_publisher_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> unitree_arm_sdk_subscriber_;
    unitree_hg::msg::dds_::LowCmd_ unitree_msg_;
    unitree_hg::msg::dds_::LowState_ unitree_msg_state_;

    // State mutex for thread safety
    std::mutex state_mtx_;

    float weight_;
    float delta_weight_{0.f};
    const float weight_rate_;
    const float kp_;
    const float kd_;
    const float dq_;
    const float tau_ff_;
    const float control_dt_;
    const float max_joint_velocity_;
    const float max_joint_delta_;
    std::chrono::milliseconds sleep_time_;

    // --- Parameters (startup/stop & gains) ---
    float startup_time_{3.0f};         // seconds
    float startup_delay_{2.0f};        // seconds
    float min_move_threshold_{0.087f}; // ~5 deg
    float stop_time_{0.8f};            // seconds

    // Gains: move/hold/transition (mirrors G1DualArmSimple)
    float kp_move_{30.f}, kd_move_{1.0f};
    float kp_hold_{6.0f}, kd_hold_{0.6f};
    float kp_transition_{15.0f}, kd_transition_{0.5f};

    // Startup poses (L then R)
    std::array<float, 7> startup_left_pose_{0, 0, 0, 0, 0, 0, 0};
    std::array<float, 7> startup_right_pose_{0, 0, 0, 0, 0, 0, 0};

    // Runtime state
    std::atomic<bool> state_received_{false};
    std::array<float, 14> current_cmd_{}; // last commanded (for decel/stop)
    std::array<float, 14> target_cmd_{};  // working target
    std::mutex cmd_mtx_;

    // --- NEW: per-joint limits & position clamps (mirror simple controller) ---
    std::array<float, 7> vel_lim_{{0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f, 0.80f}}; // rad/s
    std::array<float, 7> pos_min_{{-3.14f, -2.50f, -3.14f, -2.60f, -2.60f, -2.60f, -3.14f}};
    std::array<float, 7> pos_max_{{3.14f, 2.50f, 3.14f, 2.60f, 2.60f, 2.60f, 3.14f}};
    float weight_fixed_{0.75f}; // send every command
    int decel_steps_{15};       // last N steps decelerate

    // Updated for 7+7 DoF
    const std::array<float, 14> init_pos_{};
    const std::array<float, 14> target_pos_{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,   // left arm defaults
                                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}}; // right arm defaults

    // Joint indices for left arm (7 DoF) + right arm (7 DoF)
    const std::array<int, 14> arm_joints_{{// Left arm
                                           JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll, JointIndex::kLeftShoulderYaw,
                                           JointIndex::kLeftElbow, JointIndex::kLeftWristRoll, JointIndex::kLeftWristPitch, JointIndex::kLeftWristYaw,
                                           // Right arm
                                           JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll, JointIndex::kRightShoulderYaw,
                                           JointIndex::kRightElbow, JointIndex::kRightWristRoll, JointIndex::kRightWristPitch, JointIndex::kRightWristYaw}};

    // Separate indices for convenience
    static constexpr std::array<int, 7> idx_left{
        kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
        kLeftWristRoll, kLeftWristPitch, kLeftWristYaw};
    static constexpr std::array<int, 7> idx_right{
        kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
        kRightWristRoll, kRightWristPitch, kRightWristYaw};

    // --- NEW: per-joint limits & position clamps (mirror simple controller) ---
    // Clamp a value to [lo, hi]
    static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }

    inline std::array<float, 7> clampArray7(const std::array<float, 7> &q) const
    {
        std::array<float, 7> c{};
        for (int j = 0; j < 7; ++j)
            c[j] = clampf(q[j], pos_min_[j], pos_max_[j]);
        return c;
    }

    // Compute step count from per-joint velocity limits (like startLinearMove)
    inline int stepsFromVel(const std::array<float, 7> &from, const std::array<float, 7> &to) const
    {
        int steps = 1;
        for (int j = 0; j < 7; ++j)
        {
            const float per_step = std::max(1e-6f, vel_lim_[j] * control_dt_);
            steps = std::max(steps, static_cast<int>(std::ceil(std::fabs(to[j] - from[j]) / per_step)));
        }
        return std::max(1, steps);
    }

    // One tick toward target with deceleration over last decel_steps_
    inline bool stepToward7(std::array<float, 7> &cur, const std::array<float, 7> &tgt, int &steps)
    {
        if (steps <= 0)
            return true;
        float decel = 1.0f;
        if (steps <= decel_steps_)
        {
            float prog = static_cast<float>(steps) / static_cast<float>(decel_steps_);
            decel = 0.3f + 0.7f * prog; // 0.3→1.0
        }
        bool done = true;
        for (int j = 0; j < 7; ++j)
        {
            const float per_step = vel_lim_[j] * control_dt_ * decel;
            const float delta = clampf(tgt[j] - cur[j], -per_step, per_step);
            cur[j] = clampf(cur[j] + delta, pos_min_[j], pos_max_[j]);
            if (std::fabs(tgt[j] - cur[j]) > 1e-4f)
                done = false;
        }
        steps = done ? 0 : (steps - 1);
        return done;
    }

    // Read current measured positions into L/R arrays
    inline void read_current_lr(std::array<float, 7> &qL, std::array<float, 7> &qR)
    {
        // tracking_pos_ is 14: [L0..L6, R0..R6] (assumed order you’re filling)
        for (int j = 0; j < 7; ++j)
            qL[j] = tracking_pos_[j];
        for (int j = 0; j < 7; ++j)
            qR[j] = tracking_pos_[7 + j];
    }

    // Clamp a value to [-max_step, +max_step]
    inline float clamp_step(float v, float max_step)
    {
        if (v > max_step)
            return max_step;
        if (v < -max_step)
            return -max_step;
        return v;
    }

    // Send a full 14-DoF command with the given gains and (optionally) fixed weight
    inline void send_cmd_14(const std::array<float, 14> &q, float kp, float kd,
                            bool set_weight = true, float weight_val = 0.75f)
    {
        const int ncmd = unitree_msg_.motor_cmd().size();

        auto apply = [&](const std::array<int, 7> &idx, int q_offset)
        {
            for (int j = 0; j < 7; ++j)
            {
                int id = idx[j];
                if (id < 0 || id >= ncmd)
                    continue;
                auto &mc = unitree_msg_.motor_cmd().at(id);
                mc.q(q[q_offset + j]);
                mc.dq(dq_);
                mc.kp(kp);
                mc.kd(kd);
                mc.tau(tau_ff_);
            }
        };

        // q[0..6] -> left; q[7..13] -> right
        apply(idx_left, 0);
        apply(idx_right, 7);

        if (set_weight)
        {
            if (JointIndex::kNotUsedJoint >= 0 && JointIndex::kNotUsedJoint < ncmd)
                unitree_msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight_val);
        }
        unitree_arm_sdk_publisher_->Write(unitree_msg_);
    }
};

#endif // G1_ARM7_UNITREE_H_