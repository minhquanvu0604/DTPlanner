# Note
## To be synced also to main branch
- mantainer email


# CMakeLists.txt
OpenCV: for offset_path_plot.cpp

Haven't ported these, there could be better way :

- set_target_properties(get_next_edges_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

Check out 
- install(TARGETS...) 
- if(BUILD_TESTING)

# package.xml
- Format 3

- Erase <depend>nodelet</depend>

# d_triang_color_light_ros.hpp

- In ROS 2, all message types are scoped within their respective package namespaces and follow the msg namespace pattern, e.g., ackermann_msgs::AckermannDriveStamped -> ackermann_msgs::msg::AckermannDriveStamped

- Instead of using ros::NodeHandle, use rclcpp::Node::SharedPtr

- Subscription and publisher initialization

    - In ROS 2, Publisher is strongly-typed and make more use of smart pointers
    change ros::Publisher _pub_command_vel; to rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _pub_command_vel;

    - publishers are more tightly integrated with the nodes' lifecycle.  This is part of ROS 2’s design to support more robust and complex distributed systems, including those that require lifecycle management for nodes (using Managed Nodes).

- rate management: ROS 2 uses rclcpp::Rate similar to ROS 1, need to ensure it's integrated correctly with the asynchronous spinning mechanism (rclcpp::spin or similar)

# d_triang_color_light_ros.cpp
Only change the constructor (pub, sub init)

# utsma_path_planner.cpp