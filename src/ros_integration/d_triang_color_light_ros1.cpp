#include "d_triang_color_light_ros.hpp"


DTriangPlannerColorLightROSWrapper::DTriangPlannerColorLightROSWrapper(ros::NodeHandle nh)
    : _path_planning_rate(PATH_PLANNING_RATE), _visualise{true}
{

    _nh = nh;

    _sub_cone_array = _nh.subscribe("/ground_truth/cones", 1000, &DTriangPlannerColorLightROSWrapper::cone_array_callback,this);
    
    _pub_command_vel = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out",3,false);
    _pub_marker = _nh.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);
    _pub_steering_angle = _nh.advertise<std_msgs::UInt8MultiArray>("/steer_angle", 1);

    execution_loop();
};


void DTriangPlannerColorLightROSWrapper::cone_array_callback(const eufs_msgs::ConeArrayWithCovariance::ConstPtr& msg){
    
    std::vector<Point_2> all_visible_cones;

    std::vector<eufs_msgs::ConeWithCovariance> yellow_cones = msg->yellow_cones;
    std::vector<eufs_msgs::ConeWithCovariance> blue_cones = msg->blue_cones;

    std::vector<DTCL::Cone> cones_in;
    for (const eufs_msgs::ConeWithCovariance& cone : yellow_cones){
        Point_2 point(cone.point.x, cone.point.y);
        DTCL::Cone cone_type(point,1);
        cones_in.push_back(cone_type);
    }
    for (const eufs_msgs::ConeWithCovariance& cone : blue_cones){
        Point_2 point(cone.point.x, cone.point.y);
        DTCL::Cone cone_type(point,0);
        cones_in.push_back(cone_type);
    }
    _incoming_cones = cones_in;
}


void DTriangPlannerColorLightROSWrapper::execution_loop(){

    double max_angle_change = M_PI/(PATH_PLANNING_RATE * 18); // 1 sec 10 deg max   WHAT IS 18 ??

    while (ros::ok()){
        _dt.clear();

        visualization_msgs::MarkerArray marker_array;

        ros::spinOnce();

        if (_incoming_cones.empty()){
            _path_planning_rate.sleep();
            continue;
        }

        // set_cones_debug(_incoming_cones);
        set_cones(_incoming_cones);

        // Check the number of edges that are going to be plot, if forget to _paths.clear() will create increasing number of markers
        std::vector<std::pair<Point_2, Point_2>> plotting_edges = get_edges_for_plotting();  
        // std::cout << "Number of edges for plotting: " << plotting_edges.size() << std::endl;


        std::vector<Point_2> best_path = get_ultimate_path(); 

        Point_2 lookahead_pt = find_lookahead_point();

        double angle_to_goal = compute_orientation(Point_2(0,0), lookahead_pt);
        // std::cout << "ANGLE DIFF:  " << angle_to_goal << std::endl;
        // std::cout << "LAST ANGLE:  " << _last_angle << std::endl;


        // Fixing the angle - should be inside model plugin to represent car response
        double angle_changed = angle_to_goal - _last_angle;
        

        if (angle_changed > max_angle_change){         
            angle_to_goal = _last_angle + max_angle_change;
            std::cout << "\033[33mFixing Angle < \033[0m" << std::endl;
        } 
        if (angle_changed < -max_angle_change){
            angle_to_goal = _last_angle - max_angle_change;
            std::cout << "\033[33mFixing Angle < \033[0m" << std::endl;
        }
        std::cout << "FIXED ANGLE:  " << angle_to_goal << std::endl;

        _last_angle = angle_to_goal;    

        /////
        double angle_to_goal_deg = (angle_to_goal * 180 / M_PI) * 1.5; // WHY 1.5 ??
        auto ackerman_msg = create_ackermann_drive_stamped_msg(angle_to_goal,CAR_ACCELERATION);
        _pub_command_vel.publish(ackerman_msg);
        auto control_msg = create_steering_msg(angle_to_goal_deg);        
        _pub_steering_angle.publish(control_msg);
        ////

        if (_visualise){
            visualization_msgs::MarkerArray edges_marker_array = create_triangulation_edge_marker_array(plotting_edges);
            marker_array.markers.insert(marker_array.markers.end(), edges_marker_array.markers.begin(), edges_marker_array.markers.end());        

            visualization_msgs::Marker best_path_marker = create_path_marker(best_path, 0.0, 1.0, 0.0, 0.5);
            marker_array.markers.push_back(best_path_marker);        

            visualization_msgs::Marker lookahead_pt_marker = create_lookahead_point_marker(lookahead_pt);
            marker_array.markers.push_back(lookahead_pt_marker);

            _pub_marker.publish(marker_array);            
        }
        
        _path_planning_rate.sleep();
    }
}


ackermann_msgs::AckermannDriveStamped DTriangPlannerColorLightROSWrapper::create_ackermann_drive_stamped_msg(float steering_angle, float acceleration) {
    // Create an AckermannDrive message
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.steering_angle = steering_angle;
    drive_msg.acceleration = acceleration;

    // Create a Header message with the current timestamp
    std_msgs::Header header_msg;
    header_msg.stamp = ros::Time::now();

    // Create an AckermannDriveStamped message
    ackermann_msgs::AckermannDriveStamped ackermann_drive_stamped_msg;
    ackermann_drive_stamped_msg.header = header_msg;
    ackermann_drive_stamped_msg.drive = drive_msg;

    return ackermann_drive_stamped_msg;
}


std_msgs::UInt8MultiArray DTriangPlannerColorLightROSWrapper::create_steering_msg(int8_t steering_angle)
{
    // Map the steering angle from [-30, 30] to [0, 255]
    uint8_t mapped_steering = static_cast<uint8_t>((steering_angle + 30) * (255.0 / 60.0));

    double ddd = (steering_angle + 30) * (255.0 / 60.0);
    std::cout << "mapped steering value: " << ddd << std::endl;

    std_msgs::UInt8MultiArray control_msg;
    control_msg.data.push_back(mapped_steering);

    // Assuming braking_value is also in the range [0, 255]
    // control_msg.data.push_back(braking_value);

    return control_msg;
}



//////// Visualising //////////////////////////////////
visualization_msgs::MarkerArray DTriangPlannerColorLightROSWrapper::create_triangulation_edge_marker_array(const std::vector<std::pair<Point_2, Point_2>>& edges) {
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;

    for (const auto& edge : edges) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint"; 
        marker.ns = "triangulation_edge";
        marker.id = marker_id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.lifetime = ros::Duration(1/PATH_PLANNING_RATE);
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.02; // Line width
        marker.color.r = 1.0; // Red color
        marker.color.a = 0.8; // Fully opaque

        geometry_msgs::Point point_start, point_end;
        point_start.x = CGAL::to_double(edge.first.x());
        point_start.y = CGAL::to_double(edge.first.y());
        point_start.z = 0.0; // Assuming 2D points
        point_end.x = CGAL::to_double(edge.second.x());
        point_end.y = CGAL::to_double(edge.second.y());
        point_end.z = 0.0; // Assuming 2D points

        // Add the start and end points to the marker points
        marker.points.push_back(point_start);
        marker.points.push_back(point_end);

        markers.markers.push_back(marker);
    }

    return markers;
}

visualization_msgs::Marker DTriangPlannerColorLightROSWrapper::create_path_marker(const std::vector<Point_2>& path, double red, double green, double blue, double alpha) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.ns = "path_marker";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(1/PATH_PLANNING_RATE);

    marker.scale.x = 0.1; // Line width
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Iterate through the path and add the points to the marker as points
    for (const auto& point : path) {
        geometry_msgs::Point marker_point;
        marker_point.x = CGAL::to_double(point.x());
        marker_point.y = CGAL::to_double(point.y());
        marker_point.z = 0.0; // Assuming 2D points

        // Add the point to the marker points
        marker.points.push_back(marker_point);
    }

    return marker;
}

visualization_msgs::Marker DTriangPlannerColorLightROSWrapper::create_lookahead_point_marker(Point_2 lookahead_pt) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lookahead_point";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1/PATH_PLANNING_RATE);

    marker.pose.position.x = CGAL::to_double(lookahead_pt.x());
    marker.pose.position.y = CGAL::to_double(lookahead_pt.y());
    marker.pose.position.z = 0; 

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.9; 

    return marker;
}