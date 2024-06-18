#ifndef D_TRIANG_PLANNER_COLOR_LIGHT_ROS_H
#define D_TRIANG_PLANNER_COLOR_LIGHT_ROS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eufs_msgs/ConeArrayWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8MultiArray.h>


#include "d_triang_planner_color_light.hpp"
#include "ackermann_msgs/AckermannDriveStamped.h"

/*
The car is in the main thread
*/
class DTriangPlannerColorLightROSWrapper : public DTriangPlannerColorLight {

public:
    DTriangPlannerColorLightROSWrapper(ros::NodeHandle nh);

    void cone_array_callback(const eufs_msgs::ConeArrayWithCovariance::ConstPtr& msg);

    void execution_loop();


private:
    ackermann_msgs::AckermannDriveStamped create_ackermann_drive_stamped_msg(float steering_angle, float acceleration);
    std_msgs::UInt8MultiArray create_steering_msg(int8_t steering_angle);


    visualization_msgs::MarkerArray create_triangulation_edge_marker_array(const std::vector<std::pair<Point_2, Point_2>>& edges);
    visualization_msgs::Marker create_path_marker(const std::vector<Point_2>& path, double red, double green, double blue, double alpha);
    visualization_msgs::Marker create_lookahead_point_marker(Point_2 lookahead_pt);


private:
    ros::NodeHandle _nh;

    ros::Subscriber _sub_cone_array;

    ros::Publisher _pub_command_vel;
    ros::Publisher _pub_marker;
    ros::Publisher _pub_steering_angle;

    ros::Rate _path_planning_rate;

    bool _visualise;

    // Data
    std::vector<DTCL::Cone> _incoming_cones;
};

#endif