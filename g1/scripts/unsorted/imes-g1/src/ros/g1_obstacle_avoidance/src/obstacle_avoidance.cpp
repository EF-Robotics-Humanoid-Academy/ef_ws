// Software License Agreement (BSD)
// © 2025 MYBOTSHOP GmbH. All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <g1_interface/srv/g1_modes.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

namespace Colors
{
    const std::string RESET = "\033[0m";
    const std::string DIM = "\033[2m";
    const std::string BOLD = "\033[1m";
    const std::string RED = "\033[31m";
    const std::string GREEN = "\033[32m";
    const std::string YELLOW = "\033[33m";
    const std::string BLUE = "\033[34m";
    const std::string MAGENTA = "\033[35m";
    const std::string CYAN = "\033[36m";
    const std::string WHITE = "\033[37m";
}

class SimpleObstacleAvoidance : public rclcpp::Node
{
public:
    SimpleObstacleAvoidance() : Node("simple_obstacle_avoidance")
    {
        // Parameters
        this->declare_parameter("laser_topic", "scan");
        this->declare_parameter("cmd_vel_input_topic", "unfiltered/steamdeck_joy_teleop/cmd_vel");
        this->declare_parameter("cmd_vel_output_topic", "steamdeck_joy_teleop/cmd_vel");
        this->declare_parameter("stop_radius_m", 1.0); // STOP zone radius
        this->declare_parameter("avoidance_enabled", true);
        this->declare_parameter("log_colors", true);
        this->declare_parameter("check_timer_hz", 20.0);
        this->declare_parameter("laser_yaw_offset_deg", 0.0);
        this->declare_parameter("publish_markers", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("marker_height", 0.1);

        // Slow zone + speed cap
        this->declare_parameter("slow_zone_radius_m", 0.0); // if 0, set to 1.5 * stop
        this->declare_parameter("slow_speed_mps", 0.15);    // cap per axis inside slow zone

        // RViz overlap control
        this->declare_parameter("slow_marker_style", std::string("annulus")); // "annulus" | "stacked"
        this->declare_parameter("slow_marker_inner_margin_m", 0.02);          // annulus gap after stop radius
        this->declare_parameter("slow_marker_z_offset", -0.02);               // used if style == "stacked"

        // Get parameters
        laser_topic_ = this->get_parameter("laser_topic").as_string();
        cmd_input_topic_ = this->get_parameter("cmd_vel_input_topic").as_string();
        cmd_output_topic_ = this->get_parameter("cmd_vel_output_topic").as_string();
        stop_radius_ = this->get_parameter("stop_radius_m").as_double();
        avoidance_enabled_ = this->get_parameter("avoidance_enabled").as_bool();
        log_colors_ = this->get_parameter("log_colors").as_bool();
        double timer_hz = this->get_parameter("check_timer_hz").as_double();
        laser_yaw_offset_ = this->get_parameter("laser_yaw_offset_deg").as_double() * M_PI / 180.0;
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        marker_height_ = this->get_parameter("marker_height").as_double();

        slow_zone_radius_ = this->get_parameter("slow_zone_radius_m").as_double();
        if (slow_zone_radius_ <= 0.0)
            slow_zone_radius_ = stop_radius_ * 1.5;
        if (slow_zone_radius_ <= stop_radius_)
        {
            RCLCPP_WARN(this->get_logger(), "slow_zone_radius_m (%.2f) <= stop_radius_m (%.2f); increasing slow radius.",
                        slow_zone_radius_, stop_radius_);
            slow_zone_radius_ = stop_radius_ + 0.05;
        }
        slow_speed_mps_ = this->get_parameter("slow_speed_mps").as_double();

        slow_marker_style_ = this->get_parameter("slow_marker_style").as_string();
        slow_marker_inner_margin_m_ = this->get_parameter("slow_marker_inner_margin_m").as_double();
        slow_marker_z_offset_ = this->get_parameter("slow_marker_z_offset").as_double();

        // Publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_output_topic_, 10);
        if (publish_markers_)
        {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_sectors", 10);
        }
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SimpleObstacleAvoidance::laser_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_input_topic_, 10,
            std::bind(&SimpleObstacleAvoidance::cmd_vel_callback, this, std::placeholders::_1));

        // Service for enable/disable
        mode_service_ = this->create_service<g1_interface::srv::G1Modes>(
            "obstacle_avoidance_mode",
            std::bind(&SimpleObstacleAvoidance::mode_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Timer for checking obstacles
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / timer_hz)),
            std::bind(&SimpleObstacleAvoidance::check_obstacles, this));

        // Initialize sector flags (stop + slow)
        sector_stop_.assign(6, false);
        sector_slow_.assign(6, false);

        // Log initialization
        std::string mode_str = avoidance_enabled_ ? "ENABLED" : "DISABLED";
        RCLCPP_INFO(this->get_logger(),
                    "Simple obstacle avoidance %s. STOP=%.2fm, SLOW=%.2fm, slow_cap=%.2fm/s, yaw_offset=%.1f deg",
                    mode_str.c_str(), stop_radius_, slow_zone_radius_, slow_speed_mps_,
                    laser_yaw_offset_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "RViz slow marker style: %s (gap=%.02fm, z_offset=%.02fm)",
                    slow_marker_style_.c_str(), slow_marker_inner_margin_m_, slow_marker_z_offset_);
    }

private:
    // Parameters
    std::string laser_topic_, cmd_input_topic_, cmd_output_topic_;
    double stop_radius_;      // STOP ring outer radius
    double slow_zone_radius_; // SLOW ring outer radius
    double slow_speed_mps_;   // cap for |velocity component| inside SLOW ring
    double laser_yaw_offset_; // radians
    bool avoidance_enabled_;
    bool log_colors_;
    bool publish_markers_;
    std::string base_frame_;
    double marker_height_;
    std::string slow_marker_style_;     // "annulus" or "stacked"
    double slow_marker_inner_margin_m_; // annulus gap after stop radius
    double slow_marker_z_offset_;       // used if "stacked"

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<g1_interface::srv::G1Modes>::SharedPtr mode_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::vector<bool> sector_stop_; // inner ring occupancy
    std::vector<bool> sector_slow_; // outer ring occupancy
    geometry_msgs::msg::Twist latest_cmd_;

    // Sector definitions (base frame)
    const std::vector<double> SECTOR_ANGLES = {
        0.0,          // FORWARD
        M_PI / 3,     // FORWARD-LEFT
        -M_PI / 3,    // FORWARD-RIGHT
        M_PI,         // BACKWARD
        2 * M_PI / 3, // BACKWARD-LEFT
        -2 * M_PI / 3 // BACKWARD-RIGHT
    };

    const std::vector<std::string> SECTOR_NAMES = {
        "FORWARD", "FORWARD-LEFT", "FORWARD-RIGHT",
        "BACKWARD", "BACKWARD-LEFT", "BACKWARD-RIGHT"};

    static double normalize_angle(double a)
    {
        while (a > M_PI)
            a -= 2 * M_PI;
        while (a < -M_PI)
            a += 2 * M_PI;
        return a;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_cmd_ = *msg;

        if (!avoidance_enabled_)
        {
            cmd_vel_pub_->publish(*msg);
            return;
        }

        if (latest_scan_)
        {
            check_obstacles();
        }

        geometry_msgs::msg::Twist safe_cmd = apply_avoidance(*msg);
        cmd_vel_pub_->publish(safe_cmd);
    }

    void mode_service_callback(
        const std::shared_ptr<g1_interface::srv::G1Modes::Request> request,
        std::shared_ptr<g1_interface::srv::G1Modes::Response> response)
    {
        std::string data = request->request_data;
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);

        if (data == "on" || data == "enable" || data == "enabled" || data == "true" || data == "1")
        {
            avoidance_enabled_ = true;
            response->success = true;
            response->reason = "Obstacle avoidance enabled.";
            RCLCPP_WARN(this->get_logger(), "%sService: obstacle avoidance ENABLED.%s",
                        log_colors_ ? Colors::GREEN.c_str() : "", log_colors_ ? Colors::RESET.c_str() : "");
        }
        else if (data == "off" || data == "disable" || data == "disabled" || data == "false" || data == "0")
        {
            avoidance_enabled_ = false;
            response->success = true;
            response->reason = "Obstacle avoidance disabled (pass-through).";
            RCLCPP_WARN(this->get_logger(), "%sService: obstacle avoidance DISABLED (pass-through).%s",
                        log_colors_ ? Colors::RED.c_str() : "", log_colors_ ? Colors::RESET.c_str() : "");
        }
        else if (data == "status" || data == "state")
        {
            response->success = true;
            response->reason = std::string("Obstacle avoidance is ") +
                               (avoidance_enabled_ ? "ENABLED" : "DISABLED") + ".";
        }
        else
        {
            response->success = false;
            response->reason = "Unrecognized request_data. Use: 'on'|'enable' or 'off'|'disable' (also 'status').";
        }
    }

    void check_obstacles()
    {
        if (!latest_scan_)
            return;

        std::fill(sector_stop_.begin(), sector_stop_.end(), false);
        std::fill(sector_slow_.begin(), sector_slow_.end(), false);

        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i)
        {
            double range = latest_scan_->ranges[i];

            if (!std::isfinite(range) ||
                range < latest_scan_->range_min ||
                range > latest_scan_->range_max)
            {
                continue;
            }

            // angle in base frame
            double angle_laser = latest_scan_->angle_min + i * latest_scan_->angle_increment;
            double angle_base = normalize_angle(angle_laser + laser_yaw_offset_);

            // SLOW zone (outer ring)
            if (range <= slow_zone_radius_)
            {
                mark_sector(sector_slow_, angle_base);
            }
            // STOP zone (inner ring)
            if (range <= stop_radius_)
            {
                mark_sector(sector_stop_, angle_base);
            }
        }

        if (publish_markers_)
        {
            publish_sector_markers();
        }
    }

    void mark_sector(std::vector<bool> &sector_flags, double angle_base)
    {
        const double SECTOR_WIDTH = M_PI / 3; // ±30°
        for (size_t i = 0; i < SECTOR_ANGLES.size(); ++i)
        {
            double sector_center = SECTOR_ANGLES[i];
            double angle_diff = normalize_angle(angle_base - sector_center);
            if (std::abs(angle_diff) <= SECTOR_WIDTH / 2)
            {
                sector_flags[i] = true;
            }
        }
    }

    geometry_msgs::msg::Twist apply_avoidance(const geometry_msgs::msg::Twist &cmd)
    {
        geometry_msgs::msg::Twist out = cmd;

        // Stop/slow checks (per axis)
        bool fwd_stop = (cmd.linear.x > 0.01) && (sector_stop_[0] || sector_stop_[1] || sector_stop_[2]);
        bool fwd_slow = (cmd.linear.x > 0.01) && (sector_slow_[0] || sector_slow_[1] || sector_slow_[2]);

        bool bwd_stop = (cmd.linear.x < -0.01) && (sector_stop_[3] || sector_stop_[4] || sector_stop_[5]);
        bool bwd_slow = (cmd.linear.x < -0.01) && (sector_slow_[3] || sector_slow_[4] || sector_slow_[5]);

        bool left_stop = (cmd.linear.y > 0.01) && (sector_stop_[1] || sector_stop_[4]);
        bool left_slow = (cmd.linear.y > 0.01) && (sector_slow_[1] || sector_slow_[4]);

        bool right_stop = (cmd.linear.y < -0.01) && (sector_stop_[2] || sector_stop_[5]);
        bool right_slow = (cmd.linear.y < -0.01) && (sector_slow_[2] || sector_slow_[5]);

        // X axis
        if (fwd_stop || bwd_stop)
        {
            out.linear.x = 0.0;
            log_blocking("STOP: X blocked");
        }
        else if (fwd_slow || bwd_slow)
        {
            double sign = (cmd.linear.x >= 0.0) ? 1.0 : -1.0;
            out.linear.x = sign * std::min(std::abs(cmd.linear.x), slow_speed_mps_);
        }

        // Y axis
        if (left_stop || right_stop)
        {
            out.linear.y = 0.0;
            log_blocking("STOP: Y blocked");
        }
        else if (left_slow || right_slow)
        {
            double sign = (cmd.linear.y >= 0.0) ? 1.0 : -1.0;
            out.linear.y = sign * std::min(std::abs(cmd.linear.y), slow_speed_mps_);
        }

        // Angular unchanged; extend similarly if desired.
        return out;
    }

    void log_blocking(const std::string &message)
    {
        std::vector<std::string> stop_s, slow_s;
        for (size_t i = 0; i < SECTOR_NAMES.size(); ++i)
        {
            if (sector_stop_[i])
                stop_s.push_back(SECTOR_NAMES[i]);
            if (sector_slow_[i])
                slow_s.push_back(SECTOR_NAMES[i]);
        }

        auto join = [](const std::vector<std::string> &v)
        {
            std::string s;
            for (size_t i = 0; i < v.size(); ++i)
            {
                if (i)
                    s += ", ";
                s += v[i];
            }
            return s;
        };

        if (!stop_s.empty() || !slow_s.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "%s🛡️ %s%s - STOP:[%s]  SLOW:[%s]%s",
                        log_colors_ ? Colors::BOLD.c_str() : "",
                        message.c_str(),
                        log_colors_ ? Colors::RESET.c_str() : "",
                        join(stop_s).c_str(),
                        join(slow_s).c_str(),
                        log_colors_ ? Colors::RESET.c_str() : "");
        }
    }

    // ---------- RViz Markers ----------
    void add_ring_marker_annulus(const std::vector<bool> &flags,
                                 double inner_radius,
                                 double outer_radius,
                                 const std::string &ns,
                                 float r_on, float g_on, float b_on,
                                 float r_off, float g_off, float b_off,
                                 float alpha,
                                 visualization_msgs::msg::MarkerArray &arr)
    {
        const double SECTOR_WIDTH = M_PI / 3; // 60°
        const int arc_points = 20;            // smoother annulus

        for (size_t i = 0; i < SECTOR_ANGLES.size(); ++i)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = base_frame_;
            m.header.stamp = this->get_clock()->now();
            m.ns = ns;
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;

            if (flags[i])
            {
                m.color.r = r_on;
                m.color.g = g_on;
                m.color.b = b_on;
            }
            else
            {
                m.color.r = r_off;
                m.color.g = g_off;
                m.color.b = b_off;
            }
            m.color.a = alpha;

            m.scale.x = m.scale.y = m.scale.z = 1.0;

            double center = SECTOR_ANGLES[i];
            double start = center - SECTOR_WIDTH / 2;
            double end = center + SECTOR_WIDTH / 2;

            // Build an annulus wedge between inner_radius and outer_radius; top & bottom faces
            for (int j = 0; j < arc_points; ++j)
            {
                double a1 = start + j * (end - start) / arc_points;
                double a2 = start + (j + 1) * (end - start) / arc_points;

                // Bottom ring
                geometry_msgs::msg::Point in1, out1, in2, out2;
                in1.x = inner_radius * std::cos(a1);
                in1.y = inner_radius * std::sin(a1);
                in1.z = 0.0;
                out1.x = outer_radius * std::cos(a1);
                out1.y = outer_radius * std::sin(a1);
                out1.z = 0.0;
                in2.x = inner_radius * std::cos(a2);
                in2.y = inner_radius * std::sin(a2);
                in2.z = 0.0;
                out2.x = outer_radius * std::cos(a2);
                out2.y = outer_radius * std::sin(a2);
                out2.z = 0.0;

                // Bottom face (two triangles)
                m.points.push_back(in1);
                m.points.push_back(out1);
                m.points.push_back(out2);
                m.points.push_back(in1);
                m.points.push_back(out2);
                m.points.push_back(in2);

                // Top face (z = marker_height_) reverse winding for normal
                geometry_msgs::msg::Point in1t = in1, out1t = out1, in2t = in2, out2t = out2;
                in1t.z = out1t.z = in2t.z = out2t.z = marker_height_;

                m.points.push_back(in1t);
                m.points.push_back(out2t);
                m.points.push_back(out1t);
                m.points.push_back(in1t);
                m.points.push_back(in2t);
                m.points.push_back(out2t);
            }

            arr.markers.push_back(m);
        }
    }

    void add_ring_marker_fan(const std::vector<bool> &flags,
                             double radius,
                             const std::string &ns,
                             float r_on, float g_on, float b_on,
                             float r_off, float g_off, float b_off,
                             float alpha,
                             double z_offset,
                             visualization_msgs::msg::MarkerArray &arr)
    {
        const double SECTOR_WIDTH = M_PI / 3; // 60°
        const int arc_points = 10;

        for (size_t i = 0; i < SECTOR_ANGLES.size(); ++i)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = base_frame_;
            m.header.stamp = this->get_clock()->now();
            m.ns = ns;
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;

            if (flags[i])
            {
                m.color.r = r_on;
                m.color.g = g_on;
                m.color.b = b_on;
            }
            else
            {
                m.color.r = r_off;
                m.color.g = g_off;
                m.color.b = b_off;
            }
            m.color.a = alpha;

            m.scale.x = m.scale.y = m.scale.z = 1.0;

            double center = SECTOR_ANGLES[i];
            double start = center - SECTOR_WIDTH / 2;
            double end = center + SECTOR_WIDTH / 2;

            geometry_msgs::msg::Point c;
            c.x = 0.0;
            c.y = 0.0;
            c.z = z_offset;

            for (int j = 0; j < arc_points; ++j)
            {
                double a1 = start + j * (end - start) / arc_points;
                double a2 = start + (j + 1) * (end - start) / arc_points;

                geometry_msgs::msg::Point p1, p2;
                p1.x = radius * std::cos(a1);
                p1.y = radius * std::sin(a1);
                p1.z = z_offset;
                p2.x = radius * std::cos(a2);
                p2.y = radius * std::sin(a2);
                p2.z = z_offset;

                // Bottom fan
                m.points.push_back(c);
                m.points.push_back(p1);
                m.points.push_back(p2);

                // Top fan (extrude)
                geometry_msgs::msg::Point ct = c, p1t = p1, p2t = p2;
                ct.z += marker_height_;
                p1t.z += marker_height_;
                p2t.z += marker_height_;
                m.points.push_back(ct);
                m.points.push_back(p2t);
                m.points.push_back(p1t);
            }

            arr.markers.push_back(m);
        }
    }

    void publish_sector_markers()
    {
        visualization_msgs::msg::MarkerArray arr;

        // STOP ring: fan from origin to stop_radius_ (no overlap issues)
        add_ring_marker_fan(sector_stop_, stop_radius_, "stop_sectors",
                            /*on*/ 1.0f, 0.0f, 0.0f, /*off*/ 0.0f, 1.0f, 0.0f, 0.35f,
                            /*z_offset*/ 0.0, arr);

        // SLOW ring: either annulus (starts after stop) or z-stacked fan
        if (slow_marker_style_ == "stacked")
        {
            add_ring_marker_fan(sector_slow_, slow_zone_radius_, "slow_sectors",
                                /*on*/ 1.0f, 1.0f, 0.0f, /*off*/ 0.5f, 0.5f, 0.5f, 0.20f,
                                /*z_offset*/ slow_marker_z_offset_, arr);
        }
        else
        { // "annulus" (default)
            double inner = stop_radius_ + std::max(0.0, slow_marker_inner_margin_m_);
            double outer = slow_zone_radius_;
            if (outer <= inner)
            {
                outer = inner + 0.01;
            } // ensure > 0 thickness
            add_ring_marker_annulus(sector_slow_, inner, outer, "slow_sectors",
                                    /*on*/ 1.0f, 1.0f, 0.0f, /*off*/ 0.5f, 0.5f, 0.5f, 0.20f,
                                    arr);
        }

        marker_pub_->publish(arr);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleObstacleAvoidance>();
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("simple_obstacle_avoidance"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
