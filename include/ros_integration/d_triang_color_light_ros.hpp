#ifndef D_TRIANG_PLANNER_COLOR_LIGHT_ROS_H
#define D_TRIANG_PLANNER_COLOR_LIGHT_ROS_H

// Update headers for ROS 2
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>  // Update this to match ROS 2 message structures
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "d_triang_planner_color_light.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"  // Update the path based on ROS 2 package

/*
The car is in the main thread
*/
class DTriangPlannerColorLightROSWrapper : public DTriangPlannerColorLight {
public:
    explicit DTriangPlannerColorLightROSWrapper(const rclcpp::Node::SharedPtr& node);

private:
    void cone_array_callback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
    void execution_loop();

    ackermann_msgs::msg::AckermannDriveStamped create_ackermann_drive_stamped_msg(float steering_angle, float acceleration);
    std_msgs::msg::UInt8MultiArray create_steering_msg(int8_t steering_angle);

    visualization_msgs::msg::MarkerArray create_triangulation_edge_marker_array(const std::vector<std::pair<Point_2, Point_2>>& edges);
    visualization_msgs::msg::Marker create_path_marker(const std::vector<Point_2>& path, double red, double green, double blue, double alpha);
    visualization_msgs::msg::Marker create_lookahead_point_marker(Point_2 lookahead_pt);

private:
    rclcpp::Node::SharedPtr _node;

    rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr _sub_cone_array;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _pub_command_vel;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_marker;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _pub_steering_angle;

    rclcpp::Rate _path_planning_rate;

    bool _visualise;

    // Data
    std::vector<DTCL::Cone> _incoming_cones;
};

#endif
