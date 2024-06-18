#include "d_triang_color_light_ros.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("utsma_path_planner");

    // Create an instance of the ROS wrapper class, passing the node handle
    auto utsma_path_planner = std::make_shared<DTriangPlannerColorLightROSWrapper>(node);

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
