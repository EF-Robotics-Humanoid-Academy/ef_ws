#ifndef G1_SCAN_H
#define G1_SCAN_H

#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class G1Scanner : public rclcpp::Node
{
public:
    G1Scanner();

private:
    // ROS2
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
    rclcpp::TimerBase::SharedPtr timer_scan_;

    // Parameters
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string param_target_frame_;
    std::string param_pcd_sub_;
    std::string param_scan_pub_;
    double param_min_height_;
    double param_max_height_;
    double param_scan_time_;
    double param_range_min_;
    double param_range_max_;
    bool param_use_inf_;
    double param_inf_epsilon_;
    double param_angle_increment_;
    int param_point_clouds_combine_;
    double param_transform_tolerance_;
    int param_publisher_freq_;
    std::vector<sensor_msgs::msg::PointCloud2> last_point_clouds_;

    // Functions
    void update_scan();
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::string colorize(const std::string &text, const std::string &color) const;
};

#endif // G1_SCAN_H