#include "d_triang_realtime.hpp"

bool DTRealTime::plan_one_step(const std::vector<Point_2>& cones){

    bool match = match_new_cones(cones);

    ///////////////////////////////////
    if (match)
        std::cout << "MATCHED -------------------------------" << std::endl;
    else    
        std::cout << "NO MATCHED -------------------------------" << std::endl;
    ///////////////////////////////////

    if (match) {
        reuse_path();
    }
    else {
        plan_from_scratch();

    }

    return match;
}

/*
Not only reduces computation but also rejects bad cone pattern readings 

FALSE POSITIVE

FALSE NEGATIVE

*/
bool DTRealTime::match_new_cones(const std::vector<Point_2>& new_cones){

    // If the last step saw too few cones when just recompute
    if (_current_cones.size() < 4){
        _current_cones = new_cones;
        // _cones.clear();
        return false;
    }

    int last_cone_in_critical_zone = 0;
    int new_cone_in_critical_zone = 0;

    for (const auto& new_cone : new_cones) {
        double car_to_new_cone_dist_sq = pow(CGAL::to_double(new_cone.x()),2) + pow(CGAL::to_double(new_cone.y()),2);
        bool inside_critical_zone = car_to_new_cone_dist_sq < CRITICAL_RANGE_SQ;
        if (inside_critical_zone) 
            new_cone_in_critical_zone++;
    }

    for (const auto& last_cone : _current_cones){
        // Distance from car to a last cone
        double car_to_last_cone_dist_sq = pow(CGAL::to_double(last_cone.x()),2) + pow(CGAL::to_double(last_cone.y()),2);
        bool inside_critical_zone = car_to_last_cone_dist_sq < CRITICAL_RANGE_SQ;
        
        if (!inside_critical_zone)
            continue;
        last_cone_in_critical_zone++;

        // Find corresponding cones in the new pattern
        // Find nearest new cone
        Point_2 closest_cone;
        double min_dist_sq = std::numeric_limits<double>::max();        
        
        
        for (const auto& new_cone : new_cones) {

            // Distance from that last cone to one new cone
            double last_cone_to_new_dist_sq = CGAL::to_double(CGAL::squared_distance(last_cone, new_cone));           
            if (last_cone_to_new_dist_sq < min_dist_sq) {
                min_dist_sq = last_cone_to_new_dist_sq;
                closest_cone = new_cone;
            }
        }

        // If any last cone doesn't have a corresponding new cone
        if (min_dist_sq > MATCHING_THRESHOLD){
            // _cones.clear();
            _current_cones = new_cones;
            return false; // New patter doesn't match
        }               
    }

    _current_cones = new_cones;

    // If found new cone in the critical zone, compute triangulation again
    if (last_cone_in_critical_zone < new_cone_in_critical_zone){
        return false;
    }
    
    // Matched    
    return true;
}

void DTRealTime::plan_from_scratch(){

    // std::cout << "NUMBER OF CONES TO PLAN FROM SCRATCH: " << _current_cones.size() << std::endl;
    // print_path(_current_cones);
    // for (auto it = _dt.finite_vertices_begin(); it != _dt.finite_vertices_end(); ++it) {
    //     const Point_2& vertex = it->point();
    //     std::cout << "Vertex: (" << CGAL::to_double(vertex.x()) << ", " << CGAL::to_double(vertex.y()) << ")" << std::endl;
    // }

    set_cones(_current_cones);
    plan();

    // Clear here!!
    _best_path.clear();

    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path = get_best_path_2();
    for (auto& waypoint : path){
        Point_2 point = waypoint.first;
        _best_path.push_back(point);
    }
}

void DTRealTime::reuse_path(){

    // Clear here!!
    _best_path.clear();

    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path = get_best_path_2();
    // Remove first because it doesn't have cones on its 2 sides, avoid error
    path.erase(path.begin()); 
    _best_path.push_back(Point_2(0,0));

    for (auto& waypoint : path){

        std::array<Point_2, 2> old_cones = waypoint.second;
        std::array<Point_2, 2> new_cones;

        // Loop over 2 old cones on 2 sides of the current waypoint
        for (int i = 0; i < 2; i++){
            // Find the corresponding new cone of the current old cone
            for (auto& new_cone : _current_cones){
                double old_new_dist = CGAL::to_double(CGAL::squared_distance(old_cones.at(i), new_cone));
                if (old_new_dist < MATCHING_THRESHOLD){
                    new_cones[i] = new_cone;
                }
            }
        }
        Point_2 new_waypoint = CGAL::midpoint(new_cones[0],new_cones[1]);
        _best_path.push_back(new_waypoint);
    }
}


// double DTRealTime::get_steering(){

//     // std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path = get_best_path_2();
//     print_path(_path);

//     Point_2 car_pos(0,0);
//     Point_2 goal = _path.at(1); // Goal is the midpoint of starting edge

//     double car_to_goal_dist = CGAL::to_double(CGAL::squared_distance(car_pos, goal));
//     // If too near, next point
//     if (car_to_goal_dist < MIN_DIST_GOAL)
//         goal = _path.at(2); 

//     double angle_diff = atan2(CGAL::to_double(goal.y()), CGAL::to_double(goal.x()));
//     if (angle_diff > M_PI_2)
//         throw std::runtime_error("Turning angle too large.");

//     // Convert angle different to turning angle (eufs default : -0.52 -> 0.52)


//     return angle_diff; // JUST FOR TESTING
// }



double DTRealTime::get_steering_angle(double x_goal, double y_goal) {
    
    double theta = std::atan2(y_goal, x_goal);
    double distance = std::sqrt(x_goal * x_goal + y_goal * y_goal);
    double steering_angle = std::atan2(2.0 * WHEELBASE * std::sin(theta), distance);

    return steering_angle;
}

std::vector<Point_2> DTRealTime::get_path(){
    return _best_path;
}