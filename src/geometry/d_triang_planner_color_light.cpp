#include "d_triang_planner_color_light.hpp"
#include <cmath>


bool DTriangPlannerColorLight::set_cones(const std::vector<DTCL::Cone>& cones_local){

    if (cones_local.empty())
        return false;

    std::vector<DTCL::Cone> sorted_cones = cones_local; // Assuming cones_local is a vector of const DTCL::Cone
    std::sort(sorted_cones.begin(), sorted_cones.end(), [](const DTCL::Cone& a, const DTCL::Cone& b) {
        return a.point.x() < b.point.x();
    });


    // // // GROUP 1 // // // 
    int nearest_color = sorted_cones.at(0).color;

    // Get cones of same color as the first cones
    std::vector<DTCL::Cone> group_1;
    for (int i = 0; i < sorted_cones.size(); i++){
        if (sorted_cones.at(i).color == nearest_color){
            // Filter out cones form irrelevant paths segment on the far 2 sides
            if (abs(CGAL::to_double(sorted_cones.at(i).point.y())) > TRACK_WIDTH*2 && /////////////// NEEDS TO TUNE THIS !!
                abs(CGAL::to_double(sorted_cones.at(i).point.x())) < TRACK_WIDTH*2)
                // std::cout << "\033[33m[WARNING] Outlying cones detected\033[0m" << std::endl;
                ;
            else 
                group_1.push_back(sorted_cones.at(i));
        }
        else break;
    }
    
    if (group_1.size() < 3 && sorted_cones.size() >= 3)
        group_1.clear(); // Group 1 doesn't exist    

    // path_group_1 contains at least the car point
    std::vector<Point_2> path_group_1 = process_group_1_mtc(group_1, nearest_color);
    // // // // // //

    // // // GROUP 2 // // // 
    // The rest elements from sorted_cones
    // std::cout << "group_1.size(): " << group_1.size() << std::endl;
    _group_2 = std::vector<DTCL::Cone>(sorted_cones.begin() + group_1.size(), sorted_cones.end());

    // Check if group 2 have mixed cones
    int total_color = 0;
    for (const auto& cone : _group_2){
        total_color += cone.color;
    }

    // The group 2 is full of cones of the same color, ignore
    std::vector<Point_2> path_group_2;
    if (!(total_color == 0 || total_color == _group_2.size())){ 
        if (_group_2.size() < 4)
            _group_2.clear();

        path_group_2 = process_group_2(path_group_1.back());
        if (path_group_2.empty() && path_group_1.size() == 1){
            std::cout << "\033[33m[WARNING] path_group_2 is empty\033[0m" << std::endl;
            print_cones(sorted_cones);
            // throw std::runtime_error("!");
        }        
    }

    // Combine 
    _ultimate_path = path_group_1;
    _ultimate_path.insert(_ultimate_path.end(), path_group_2.begin(), path_group_2.end());

    return true;
}

bool DTriangPlannerColorLight::set_cones_debug(const std::vector<DTCL::Cone>& cones_local){

    if (cones_local.empty())
        return false;

    std::vector<DTCL::Cone> sorted_cones = cones_local; // Assuming cones_local is a vector of const DTCL::Cone
    std::sort(sorted_cones.begin(), sorted_cones.end(), [](const DTCL::Cone& a, const DTCL::Cone& b) {
        return a.point.x() < b.point.x();
    });

    // // // GROUP 1 // // // 
    int nearest_color = sorted_cones.at(0).color;

    // Get cones of same color as the first cones
    std::vector<DTCL::Cone> group_1;
    for (int i = 0; i < sorted_cones.size(); i++){
        if (sorted_cones.at(i).color == nearest_color){
            // Filter out cones form irrelevant paths segment on the far 2 sides
            if (abs(CGAL::to_double(sorted_cones.at(i).point.y())) > TRACK_WIDTH*2 && /////////////// NEEDS TO TUNE THIS !!
                abs(CGAL::to_double(sorted_cones.at(i).point.x())) < TRACK_WIDTH*2)
                std::cout << "\033[33m[WARNING] Outlying cones detected\033[0m" << std::endl;
            else 
                group_1.push_back(sorted_cones.at(i));
        }
        else break;
    }
    
    if (group_1.size() < 3 && sorted_cones.size() >= 3)
        group_1.clear(); // Group 1 doesn't exist    

    std::cout << "Cones both groups::::::::::::::::::::::::::: " << std::endl;
    print_cones(sorted_cones);
    std::cout << "Cones group 1::::::::::::::::::::::::::: " << std::endl;
    print_cones(group_1);

    // path_group_1 contains at least the car point
    std::vector<Point_2> path_group_1 = process_group_1_mtc(group_1, nearest_color);
    
    std::cout << "Path from group 1: " << path_group_1.size() << " element(s)" << std::endl;
    print_path(path_group_1);
    // // // // // //

    // // // GROUP 2 // // // 
    // The rest elements from sorted_cones
    // std::cout << "group_1.size(): " << group_1.size() << std::endl;
    _group_2 = std::vector<DTCL::Cone>(sorted_cones.begin() + group_1.size(), sorted_cones.end());

    std::cout << "Cones group 2::::::::::::::::::::::::::: " << std::endl;
    print_cones(_group_2);


    // Check if group 2 have mixed cones
    int total_color = 0;
    for (const auto& cone : _group_2){
        total_color += cone.color;
    }

    // The group 2 is full of cones of the same color, ignore
    std::vector<Point_2> path_group_2;
    if (!(total_color == 0 || total_color == _group_2.size())){ 
        if (_group_2.size() < 4)
            _group_2.clear();

        path_group_2 = process_group_2(path_group_1.back());
        if (path_group_2.empty() && path_group_1.size() == 1){
            std::cout << "\033[33m[WARNING] path_group_2 is empty\033[0m" << std::endl;
            print_cones(sorted_cones);
            // throw std::runtime_error("!");
        }        
    }

    // Combine 
    _ultimate_path = path_group_1;
    _ultimate_path.insert(_ultimate_path.end(), path_group_2.begin(), path_group_2.end());

    return true;
}

std::vector<Point_2> DTriangPlannerColorLight::process_group_1(const std::vector<DTCL::Cone>& group_1, int nearest_color){
    
    std::vector<Point_2> path_group_1;
    Point_2 car_pt(0,0);
    path_group_1.push_back(car_pt); // Include the car position
    
    if (group_1.empty())
        return path_group_1;    

    // std::vector<Point_2> group_1_as_pt;
    // for (const auto& cone : group_1){
    //     group_1_as_pt.push_back(cone.point);
    // }    

    // Offset
    if (nearest_color == 0){
        for (auto& cone : group_1){
            Point_2 waypoint(cone.point.x() - (TRACK_WIDTH/2),cone.point.y() - (TRACK_WIDTH/2));
            path_group_1.push_back(waypoint);
        }
    }
    else if (nearest_color == 1){
        for (auto& cone : group_1){
            Point_2 waypoint(cone.point.x() - (TRACK_WIDTH/2),cone.point.y() + (TRACK_WIDTH/2));
            path_group_1.push_back(waypoint);
        }
    }
    else throw std::runtime_error("Cone color is a value other than 0 and 1");

    // std::cout << "path_group_1.size(): " << path_group_1.size() << std::endl;
    // for (auto pt : path_group_1)
    //     std::cout << pt << std::endl;

    // Find the pt to connect with car pt that complies to angle condition
    // int first_waypt_id = -1;
    for (int i = 1; i < path_group_1.size(); i++){
        
        // Angle from first car to pt in offset_line 
        double angle = abs((compute_orientation(car_pt, path_group_1.at(i))));

        // 'Preliminary condtion'
        if (angle > M_PI/4){
            path_group_1.erase(path_group_1.begin() + i);
            i--;
            continue;
        }
        else 
            break;

        // Yellow text
        std::cout << "\033[33m[WARNING] Cannot find first waypoint in group 1\033[0m" << std::endl;
    }    
    return path_group_1;
}

std::vector<Point_2> DTriangPlannerColorLight::process_group_1_mtc(const std::vector<DTCL::Cone>& group_1, int nearest_color){
    
    std::vector<Point_2> path_group_1;
    Point_2 car_pt(0,0);

    if (group_1.empty()){
        path_group_1.push_back(car_pt); // Include the car position
        return path_group_1;        
    }

    // Handle 1 cone situation
    if(group_1.size() == 1){
        path_group_1.push_back(car_pt);
        Point_2 only_pt = group_1.at(0).point;
        path_group_1.push_back(Point_2(only_pt.x(), only_pt.y() + (group_1.at(0).color ? -1 : 1) * TRACK_WIDTH/2));
        return path_group_1;
    }
    
    // std::cout << "Print points: " << std::endl;
    std::vector<Point_2> cone_as_pt;
    for (auto cone : group_1){
        cone_as_pt.push_back(cone.point);
        // std::cout << cone.point << std::endl;
    }

    int cone_color = group_1.at(0).color;
    bool is_offset_left;
    if (cone_color)
        is_offset_left = true;
    else 
        is_offset_left = false;

    path_group_1 = offset_polyline(cone_as_pt, TRACK_WIDTH/2, is_offset_left);

    // std::cout<< "Print offset: " << path_group_1.size() << std::endl;
    // for (const auto& p : path_group_1){
    //     std::cout << p << std::endl;
    // }

    // Connect the car to the offset line
    // Eliminate some first path segments that go in the opposite direction of the whole path 
    double leaf_y = CGAL::to_double(path_group_1.back().y());
    path_group_1.erase(
        std::remove_if(path_group_1.begin(), path_group_1.end(), [leaf_y](const Point_2& point) {
            return CGAL::to_double(point.y()) * leaf_y < 0;
        }),
        path_group_1.end()
    );

    path_group_1.insert(path_group_1.begin(), car_pt); 



    // Find the pt to connect with car pt that complies to angle condition
    // int first_waypt_id = -1;
    for (int i = 1; i < path_group_1.size(); i++){
        
        // Angle from first car to pt in offset_line 
        double angle = abs((compute_orientation(car_pt, path_group_1.at(i))));

        // 'Preliminary condtion'
        if (angle > M_PI/4){
            path_group_1.erase(path_group_1.begin() + i);
            i--;
            continue;
        }
        else 
            break;

        // Yellow text
        std::cout << "\033[33m[WARNING] Cannot find first waypoint in group 1\033[0m" << std::endl;
    }  




    return path_group_1;
}

std::vector<Point_2> DTriangPlannerColorLight::process_group_2(const Point_2& starting_pt){
    std::vector<Point_2> empty;

    if (_group_2.size() < 2)
        return empty;

    triangulate(_group_2);
    Edge first_edge = get_first_edge(starting_pt);
    
    if (first_edge == Edge())
        return empty;

    std::vector<std::vector<Point_2>> complete_paths = expand(first_edge, starting_pt);
    choose_best_path(complete_paths);

    return _best_path_group_2;
}

void DTriangPlannerColorLight::triangulate(const std::vector<DTCL::Cone>& group_2){
    std::vector<Point_2> input_vect;
    
    // DTCL
    for (auto& cone : group_2){
        input_vect.push_back(cone.point);
    }

    // Triangulation computed here
    _dt.insert(input_vect.begin(), input_vect.end()); 
}

Edge DTriangPlannerColorLight::get_first_edge(const Point_2& starting_pt){

    DelaunayTriangulation::Vertex_handle nearest, second_nearest;
    double nearest_distance = std::numeric_limits<double>::max(), second_nearest_distance = std::numeric_limits<double>::max();
    
    for (auto v = _dt.finite_vertices_begin(); v != _dt.finite_vertices_end(); ++v) {
        // Boundaries for the points
        if (abs(CGAL::to_double(v->point().y())) > 10)
            continue;
        
        double distance = CGAL::to_double(CGAL::squared_distance(v->point(), starting_pt));

        if (distance < nearest_distance) {
            nearest = v;
            nearest_distance = distance;
        }
    }
    Point_2 nearest_pt = nearest->point();
    int nearest_color = -1;

    for (const auto& cone : _group_2){
        if (is_approx_equal(nearest_pt, cone.point, 1e-3)){
            nearest_color = cone.color;
            break;
        }
    }

    if (nearest_color == -1)
        throw std::runtime_error("Can't find nearest cone color");

    // Find the second nearest vertex with a different color
    for (auto v = _dt.finite_vertices_begin(); v != _dt.finite_vertices_end(); ++v) {
        
        // Skip the nearest one
        if (is_approx_equal(v->point(), nearest_pt, 1e-3))
            continue;

        // Find color
        Point_2 current_cone_pt = v->point();
        int current_cone_color = -1;
        for (const auto& cone : _group_2){
            if (is_approx_equal(current_cone_pt, cone.point, 1e-3)){
                current_cone_color = cone.color;
            }
        }
        if (current_cone_color == -1)
            throw std::runtime_error("Can't find current cone color");
        
        if (current_cone_color != nearest_color) {
            double distance = CGAL::to_double(CGAL::squared_distance(v->point(), starting_pt));
            if (distance < second_nearest_distance) {
                second_nearest = v;
                second_nearest_distance = distance;
            }
        }
    }

    // if (second_nearest == DelaunayTriangulation::Vertex_handle())
    //     std::cout << "No vertex assigned for second nearest" << std::endl;

    // From 2 nearest vertices, find the corresponding edge
    Edge nearest_edge;
    // bool edge_found = false;

    Edge_circulator c = _dt.incident_edges(nearest), done(c);
    if (c != 0) {
        do {
            if ((c->first->vertex((c->second + 1) % 3) == second_nearest) || (c->first->vertex((c->second + 2) % 3) == second_nearest)) {
                nearest_edge = *c;
                // edge_found = true;
                break;
            }
        } while (++c != done);
    }

    // // Logic error checking
    // if (nearest_edge == Edge()) 
    //     throw std::runtime_error("Vertices are not connected by an edge");

    if (_dt.is_infinite(nearest_edge))
        throw std::runtime_error("The nearest edge is infinite");

    // std::cout << "The edge:";
    // print_edge_vertices(nearest_edge);

    return nearest_edge;
}

/*
 * 
 * NOTE: need to confirm that first_edge is not infinite 
 * 
 * POTENTIAL FEATURES
 *      Save rejected paths -> prevent the case where the good path is rejected by the last segment
*/
std::vector<std::vector<Point_2>> DTriangPlannerColorLight::expand(const Edge& first_edge, const Point_2& starting_pt) {

    std::vector<std::vector<Point_2>> complete_paths;

    // A queue to store the instantaneous traversing progress 
    // Each has 2 edges represents a state of traverse progress
    // They represent the previous and current edge respectively
    std::deque<DTCL::TraverseState> traverse_progress;

    std::shared_ptr<DTCL::Node> starting_pt_ptr = std::make_shared<DTCL::Node>(Pose(starting_pt, 0));

    std::pair<Point_2, Point_2> two_pts_first_node = get_points_from_edge(first_edge);
    Point_2 next_node_midpt = CGAL::midpoint(two_pts_first_node.first, two_pts_first_node.second);

    double next_node_angle = abs(normalize(compute_orientation(starting_pt_ptr->pose.position, next_node_midpt)));
    
    std::shared_ptr<DTCL::Node> next_node_ptr = std::make_shared<DTCL::Node>(Pose(next_node_midpt, next_node_angle));

    ///////
    int passed = 0; // Number of next nodes that pass the 'preliminary condtion'
    int num_node_checked = 0; // Total number of nodes checked
    ///////

    DTCL::TraverseState initial_state;
    initial_state.node_ptr = next_node_ptr;
    initial_state.current_edge = first_edge;
    initial_state.previous_edge = Edge();
    traverse_progress.push_back(initial_state);


    // Add another state that has the mirrored edge to first_edge
    DelaunayTriangulation::Face_handle face_m = first_edge.first;
    int index_m = first_edge.second;

    // The opposite edge is the same edge in the neighboring face
    DelaunayTriangulation::Face_handle neighbor_face = face_m->neighbor(index_m);
    int neighbor_index = _dt.mirror_index(face_m, index_m);
    Edge mirror_edge_m(neighbor_face, neighbor_index);

    DTCL::TraverseState initial_state_with_mirrored_edge;
    initial_state_with_mirrored_edge.node_ptr = next_node_ptr;
    initial_state_with_mirrored_edge.current_edge = mirror_edge_m;
    initial_state_with_mirrored_edge.previous_edge = Edge();
    traverse_progress.push_back(initial_state_with_mirrored_edge);

    bool is_first_edge = true; /////////////////////
    int first_edge_reject = 0;

    while (!traverse_progress.empty()) {

        DTCL::TraverseState current_state = traverse_progress.front();
        traverse_progress.pop_front();
        
        // Expand on the last state
        std::vector<Edge> next_edges = get_next_edges(current_state.current_edge, current_state.previous_edge);
        std::shared_ptr<DTCL::Node> last_node_ptr = current_state.node_ptr;
        Edge& current_new_edge = next_edges.at(0);
                
        // Examine 2 edges
        for (int i = 1; i < 3; i++){

            num_node_checked++;

            Edge& next_new_edge = next_edges.at(i);

            // If boundary edge, finish the tree 
            if (_dt.is_infinite(next_new_edge)){                       
                std::vector<Point_2> path = backtrack_path(last_node_ptr);
                complete_paths.push_back(path);
                
                break; // Break otherwise the same path will be pushed back twice
            }

            std::pair<Point_2, Point_2> two_pts = get_points_from_edge(next_new_edge);
            Point_2 midpoint = CGAL::midpoint(two_pts.first, two_pts.second);

            double midpoint_angle = compute_orientation(last_node_ptr->pose.position, midpoint);
            double angle_diff = abs(normalize(last_node_ptr->pose.yaw - midpoint_angle));

            // 'Preliminary condtion' to accept the next node 
            // Loose condition to reduce creating insensible paths
            // After creating all complete paths that pass this condtion, they are compared for the best one
            if (angle_diff > M_PI/4){
                if (is_first_edge){
                    if (angle_diff > M_PI){
                        continue;
                    }
                }
                else 
                    continue;                    
            }
            passed++;

            // Create new node, the node is created here and only here
            std::shared_ptr<DTCL::Node> new_node_ptr = std::make_shared<DTCL::Node>(Pose(midpoint, midpoint_angle));
            // Register this node
            // Connect this node to last node
            new_node_ptr->parent_ptr = last_node_ptr;
            last_node_ptr->children_ptrs.push_back(new_node_ptr);
            // Add cone information of this node
            // new_node_ptr->cones = {two_pts.first, two_pts.second};

            // Otherwise add to new state
            DTCL::TraverseState new_state(new_node_ptr, next_new_edge, current_new_edge);
            traverse_progress.push_back(new_state);                
        }          
        is_first_edge = false;
    }

    return complete_paths;
}

std::vector<std::vector<Point_2>> DTriangPlannerColorLight::expand_debug(const Edge& first_edge, const Point_2& starting_pt) {

    std::vector<std::vector<Point_2>> complete_paths;

    // A queue to store the instantaneous traversing progress 
    // Each has 2 edges represents a state of traverse progress
    // They represent the previous and current edge respectively
    std::deque<DTCL::TraverseState> traverse_progress;

    std::shared_ptr<DTCL::Node> starting_pt_ptr = std::make_shared<DTCL::Node>(Pose(starting_pt, 0));

    std::pair<Point_2, Point_2> two_pts_first_node = get_points_from_edge(first_edge);
    Point_2 next_node_midpt = CGAL::midpoint(two_pts_first_node.first, two_pts_first_node.second);

    double next_node_angle = abs(normalize(compute_orientation(starting_pt_ptr->pose.position, next_node_midpt)));
    
    std::shared_ptr<DTCL::Node> next_node_ptr = std::make_shared<DTCL::Node>(Pose(next_node_midpt, next_node_angle));


    // DEBUGGING
    std::cout << "FIRST NODE: " << std::endl;
    print_edge_vertices(first_edge);
    std::cout << "Midpoint: " << next_node_midpt << std::endl;
    std::cout << "Previous point: " << starting_pt_ptr->pose.position << std::endl;
    std::cout << "Angle diff: " << next_node_angle << std::endl; 
    std::cout << "=====================================" << std::endl; 
    ///////
    int passed = 0; // Number of next nodes that pass the 'preliminary condtion'
    int num_node_checked = 0; // Total number of nodes checked
    ///////

    DTCL::TraverseState initial_state;
    initial_state.node_ptr = next_node_ptr;
    initial_state.current_edge = first_edge;
    initial_state.previous_edge = Edge();
    traverse_progress.push_back(initial_state);


    // Add another state that has the mirrored edge to first_edge
    DelaunayTriangulation::Face_handle face_m = first_edge.first;
    int index_m = first_edge.second;

    // The opposite edge is the same edge in the neighboring face
    DelaunayTriangulation::Face_handle neighbor_face = face_m->neighbor(index_m);
    int neighbor_index = _dt.mirror_index(face_m, index_m);
    Edge mirror_edge_m(neighbor_face, neighbor_index);

    DTCL::TraverseState initial_state_with_mirrored_edge;
    initial_state_with_mirrored_edge.node_ptr = next_node_ptr;
    initial_state_with_mirrored_edge.current_edge = mirror_edge_m;
    initial_state_with_mirrored_edge.previous_edge = Edge();
    traverse_progress.push_back(initial_state_with_mirrored_edge);

    bool is_first_edge = true; /////////////////////
    int first_edge_reject = 0;

    while (!traverse_progress.empty()) {

        DTCL::TraverseState current_state = traverse_progress.front();
        traverse_progress.pop_front();
        
        // Expand on the last state
        std::vector<Edge> next_edges = get_next_edges(current_state.current_edge, current_state.previous_edge);
        std::shared_ptr<DTCL::Node> last_node_ptr = current_state.node_ptr;
        Edge& current_new_edge = next_edges.at(0);
        
        std::cout << "Current point: " << last_node_ptr->pose.position << std::endl;
        std::cout << "Current edge: ";
        print_edge_vertices(current_new_edge);
        
        std::cout << "-----------------------------------" << std::endl;

        // Examine 2 edges
        for (int i = 1; i < 3; i++){

            num_node_checked++;

            Edge& next_new_edge = next_edges.at(i);

            // If boundary edge, finish the tree 
            if (_dt.is_infinite(next_new_edge)){                       
                std::cout << "END PATH" << std::endl;
                std::vector<Point_2> path = backtrack_path(last_node_ptr);
                complete_paths.push_back(path);
                
                break; // Break otherwise the same path will be pushed back twice
            }

            std::pair<Point_2, Point_2> two_pts = get_points_from_edge(next_new_edge);
            Point_2 midpoint = CGAL::midpoint(two_pts.first, two_pts.second);

            double midpoint_angle = compute_orientation(last_node_ptr->pose.position, midpoint);
            double angle_diff = abs(normalize(last_node_ptr->pose.yaw - midpoint_angle));


            print_edge_vertices(next_new_edge);
            std::cout << "Midpoint: " << midpoint << std::endl;
            std::cout << "Previous point: " << last_node_ptr->pose.position << std::endl;
            std::cout << "Angle diff: " << angle_diff << std::endl; 


            // 'Preliminary condtion' to accept the next node 
            // Loose condition to reduce creating insensible paths
            // After creating all complete paths that pass this condtion, they are compared for the best one
            if (angle_diff > M_PI/4){
      
                if (is_first_edge){
                    if (angle_diff > M_PI){
                        std::cout << "REJECTED FIRST EDGE" << std::endl;
                        std::cout << "=====================================" << std::endl;
                        continue;
                    }
                }
                else {
                    std::cout << "REJECTED" << std::endl;
                    std::cout << "=====================================" << std::endl;
                    continue;                    
                }
            }

            std::cout << "PASSED" << std::endl;
            passed++;

            // Create new node, the node is created here and only here
            std::shared_ptr<DTCL::Node> new_node_ptr = std::make_shared<DTCL::Node>(Pose(midpoint, midpoint_angle));
            // Register this node
            // Connect this node to last node
            new_node_ptr->parent_ptr = last_node_ptr;
            last_node_ptr->children_ptrs.push_back(new_node_ptr);
            // Add cone information of this node
            // new_node_ptr->cones = {two_pts.first, two_pts.second};


            // Otherwise add to new state
            DTCL::TraverseState new_state(new_node_ptr, next_new_edge, current_new_edge);
            traverse_progress.push_back(new_state);                

            std::cout << "=====================================" << std::endl; 
        }          
        is_first_edge = false;

        std::cout << "-----------------------------------" << std::endl; 
    }
    std::cout << "FINISHED GENERATING PATHS" << std::endl;
    std::cout << "Passed: " << passed << std::endl;
    std::cout << "Total number of nodes checked: " << num_node_checked << std::endl;
    print_all_possible_paths(complete_paths);

    return complete_paths;
}


std::vector<Point_2> DTriangPlannerColorLight::get_ultimate_path(){
    return _ultimate_path;
}

std::vector<Point_2> DTriangPlannerColorLight::get_best_path_group_2(){
    return _best_path_group_2;
}

std::vector<std::vector<Point_2>> DTriangPlannerColorLight::get_other_paths(){
    return _other_paths_group_2;
}


/////////////////////////////////// TESTING FUNCTION ///////////////////////////////////////

void DTriangPlannerColorLight::print_all_possible_paths(std::vector<std::vector<Point_2>> paths) {
    std::cout << "Number of paths: " << paths.size() << std::endl;

    for (const auto& path : paths) {
        std::cout << "Path: ";
        for (const auto& point : path) {
            std::cout << "(" << point.x() << ", " << point.y() << ") ";
        }
        std::cout << std::endl;
    }
}

std::vector<std::pair<Point_2, Point_2>> DTriangPlannerColorLight::get_edges_for_plotting(){
    // Get the edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    for (auto it = _dt.finite_edges_begin(); it != _dt.finite_edges_end(); ++it) {
        Kernel::Segment_2 segment = _dt.segment(it);
        edges.emplace_back(segment.start(), segment.end());
    }
    return edges;
}

Point_2 DTriangPlannerColorLight::transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw) {

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}

// /////////////////// PRIVATE FUNCTIONS ///////////////////////////////////////

// /*
//  * Get next 2 edges from the current edge, moving to the next face
//  * Returns empty vector means the path meets the dead end
//  * 
//  * If the edge is a starting position of the path, previous_edge will be created by the defautl contructor
//  * 
//  * Both edge must refer to the same face (current_edge.first == previous_edge.first)
//  * 
//  * Returns 3 edges, the first one is the current_edge with respect to new _face
//  * The two other edges are others of the new_face
// */
std::vector<Edge> DTriangPlannerColorLight::get_next_edges(Edge current_edge, Edge previous_edge) {
    
    std::vector<Edge> next_edges;
    Edge edge0;
    Edge edge1;
    Edge edge2;

    // The case of first edge
    if (previous_edge.first == DelaunayTriangulation::Face_handle()) {
        // std::cout << "FIRST EDGE" << std::endl;

        DelaunayTriangulation::Face_handle new_face = current_edge.first;

        // There may be circumstances where the current_edge is not inifinite but 
        // current_edge.first is still inifinite 
        if (_dt.is_infinite(new_face)){

            int new_index = _dt.mirror_index(new_face, current_edge.second);

            new_face = current_edge.first->neighbor(current_edge.second);
            // std::cout << "new_face:" << std::endl;
            // print_face_vertices(new_face);

            // // In this case we have to find the correct edge index
            // int new_index = -1;
            // for (int i = 0; i < 3; ++i) {
            //     Edge possible_edge(new_face, i);
            //     if (are_edges_equal(possible_edge, current_edge)) {
            //         new_index = i;
            //         break;
            //     }
            // }
            // if (new_index == -1) {
            //     throw std::runtime_error("Could not find the edge in the new face.");
            // }

            

            edge0 = Edge(new_face, new_index);
            edge1 = Edge(new_face, (new_index + 1) % 3);
            edge2 = Edge(new_face, (new_index + 2) % 3);

        }
        else{
            edge0 = Edge(new_face, current_edge.second);
            edge1 = Edge(new_face, (current_edge.second + 1) % 3);
            edge2 = Edge(new_face, (current_edge.second + 2) % 3);
        }

        next_edges.push_back(edge0);
        next_edges.push_back(edge1);
        next_edges.push_back(edge2);

        // std::cout << "Print edge: " << std::endl;
        // for (auto& edge : next_edges){
        //     print_edge_vertices(edge);
        // }

        return next_edges;
    }

    // Not first edge
    // The previous face is the one that contains both current_edge and previous_edge
    DelaunayTriangulation::Face_handle previous_face = current_edge.first; // (current_edge.first == previous_edge.first)
    DelaunayTriangulation::Face_handle new_face = previous_face->neighbor(current_edge.second);
    // // Debugging
    // std::cout << "Print new_face vertices" << std::endl;
    // print_face_vertices(new_face);

    int new_face_edge_index = new_face->index(previous_face);

    edge0 = Edge(new_face, new_face_edge_index);
    edge1 = Edge(new_face, (new_face_edge_index + 1) % 3);
    edge2 = Edge(new_face, (new_face_edge_index + 2) % 3);

    next_edges.push_back(edge0);
    next_edges.push_back(edge1);
    next_edges.push_back(edge2);

    return next_edges;
}

Point_2 DTriangPlannerColorLight::find_lookahead_point() {
    if (_ultimate_path.empty()) {
        throw std::runtime_error("Path is empty");
    }

    double accumulated_distance = 0.0;
    Point_2 previous_point(0,0);

    for (const auto& point : _ultimate_path) {
        double segment_length = std::sqrt(CGAL::to_double(CGAL::squared_distance(previous_point, point)));

        // Check if we have reached or passed the lookahead distance
        if (accumulated_distance + segment_length >= LOOKAHEAD_DISTANCE) {
            double ratio = (LOOKAHEAD_DISTANCE - accumulated_distance) / segment_length;
            double lookahead_x = CGAL::to_double(previous_point.x()) + ratio * (CGAL::to_double(point.x()) - CGAL::to_double(previous_point.x()));
            double lookahead_y = CGAL::to_double(previous_point.y()) + ratio * (CGAL::to_double(point.y()) - CGAL::to_double(previous_point.y()));
            return Point_2(lookahead_x, lookahead_y);
        }

        accumulated_distance += segment_length;
        previous_point = point;
    }

    // If we reach this point, the lookahead distance extends beyond the path,
    // so return the last point on the path.
    return _ultimate_path.back();
}


///////////// UTILITY FUNCITON //////////////////////////////

bool DTriangPlannerColorLight::are_edges_equal(const Edge& e1, const Edge& e2) {
    double tolerance = 1e-3;
    auto seg1 = _dt.segment(e1);
    auto seg2 = _dt.segment(e2);

    return (is_approx_equal(seg1.source(), seg2.source(), tolerance) && is_approx_equal(seg1.target(), seg2.target(), tolerance)) ||
           (is_approx_equal(seg1.source(), seg2.target(), tolerance) && is_approx_equal(seg1.target(), seg2.source(), tolerance));
}


double DTriangPlannerColorLight::normalize(double angle){

    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }    
    return angle;
}

double DTriangPlannerColorLight::compute_orientation(const Point_2& p1, const Point_2& p2) {
    double deltaX = CGAL::to_double(p2.x()) - CGAL::to_double(p1.x());
    double deltaY = CGAL::to_double(p2.y()) - CGAL::to_double(p1.y());

    // std::cout << "deltaX: " << deltaX << std::endl;
    // std::cout << "deltaY: " << deltaY << std::endl;
    // std::cout << "std::atan2(deltaY, deltaX): " << std::atan2(deltaY, deltaX) << std::endl;

    return std::atan2(deltaY, deltaX); // Returns angle in radians
}

std::vector<Point_2> DTriangPlannerColorLight::backtrack_path(const std::shared_ptr<DTCL::Node>& leaf_node) {
    
    std::vector<Point_2> path;
    std::shared_ptr<DTCL::Node> current_node = leaf_node;


    while (current_node) {
        path.push_back(current_node->pose.position);
        current_node = current_node->parent_ptr;
    }

    // Reverse the path to get it from root to leaf
    std::reverse(path.begin(), path.end());

    return path;
}

// Important to have this 
// If index = -1, C++ exception with description "std::bad_alloc" thrown in the test body
void DTriangPlannerColorLight::choose_best_path(std::vector<std::vector<Point_2>> complete_paths){
    
    if (complete_paths.empty())
        return;

    int max_size = 0;
    int index_of_longest = -1;

    for (int i = 0; i < complete_paths.size(); ++i) {
        if (complete_paths.at(i).size() > max_size) {
            max_size = complete_paths.at(i).size();
            index_of_longest = i;
        }
    }

    // Important to have this 
    // If index = -1, C++ exception with description "std::bad_alloc" thrown in the test body
    // if (index_of_longest == -1) {
    //     throw std::runtime_error("_paths is empty");
    //     return;
    // }
        
    _best_path_group_2 = complete_paths.at(index_of_longest);

    _other_paths_group_2.clear();
    for (int i = 0; i < complete_paths.size(); ++i) {
        if (i != index_of_longest) {
            _other_paths_group_2.push_back(complete_paths.at(i));
        }
    }
}

bool DTriangPlannerColorLight::is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance) {
    return CGAL::squared_distance(p1, p2) < tolerance * tolerance;
}

// Printing utilities ///////////////

void DTriangPlannerColorLight::print_edge_vertices(const Edge& edge) {
    // Extract the face and index from the edge
    auto face = edge.first;
    int index = edge.second;

    // The vertices of the edge are the next two vertices in the face
    Point_2 vertex1 = face->vertex((index + 1) % 3)->point();
    Point_2 vertex2 = face->vertex((index + 2) % 3)->point();

    std::cout << "Edge vertices: (" 
            << vertex1 << "), (" 
            << vertex2 << ")" << std::endl;
}

std::pair<Point_2, Point_2> DTriangPlannerColorLight::get_points_from_edge(const Edge& edge) {
    auto face = edge.first;
    int i = edge.second;

    Point_2 p1 = face->vertex((i + 1) % 3)->point(); // Next vertex in the face
    Point_2 p2 = face->vertex((i + 2) % 3)->point(); // Next vertex after p1 in the face

    return std::make_pair(p1, p2);
}

void DTriangPlannerColorLight::print_cones(const std::vector<DTCL::Cone>& cones) {
    for (const auto& cone : cones) {
        std::cout << "Cone - X: " << cone.point.x() << ", Y: " << cone.point.y() << ", Color: " << cone.color << std::endl;
    }
}

void DTriangPlannerColorLight::print_path(const std::vector<Point_2>& path) {
    for (const Point_2& point : path) {
        std::cout << "Point (" << point << ")  ";
    }
    std::cout << std::endl;
}

double DTriangPlannerColorLight::cross_product(const Point_2 &a, const Point_2 &b)
{
    return CGAL::to_double(a.x() * b.y() - a.y() * b.x());
}

void DTriangPlannerColorLight::offset_segment(const Point_2 &p1, const Point_2 &p2, double offset_distance, bool is_offset_left, Point_2 &new_p1, Point_2 &new_p2)
{
    // Direction vector
    Kernel::Vector_2 dir = p2 - p1;
    // Normal vector (perpendicular)
    Kernel::Vector_2 normal = Kernel::Vector_2(-dir.y(), dir.x());

    // Depending on the direction, use the normal vector to offset the points
    double length = std::sqrt(CGAL::to_double(normal.squared_length()));
    normal = normal / length * offset_distance;

    if (!is_offset_left)
    {
        normal = -normal;
    }

    new_p1 = p1 + normal;
    new_p2 = p2 + normal;
}

Point_2 DTriangPlannerColorLight::find_point_at_distance(const Point_2& p1, const Point_2& p2, double distance, bool condition)
{
    // Calculate the squared length between p1 and p2
    double squared_length_p1_p2 = CGAL::to_double(CGAL::squared_distance(p1, p2));

    Point_2 p3;
    if (condition) {
        // P3 is at a distance from P2, in the direction of P1 to P2
        double factor = distance / std::sqrt(squared_length_p1_p2);
        p3 = Point_2(p2.x() - factor * (p2.x() - p1.x()), p2.y() - factor * (p2.y() - p1.y()));
    } else {
        // P3 is at a distance from P2, in the opposite direction of P1 to P2
        double factor = distance / std::sqrt(squared_length_p1_p2);
        p3 = Point_2(p2.x() + factor * (p2.x() - p1.x()), p2.y() + factor * (p2.y() - p1.y()));
    }
    return p3;
}

int DTriangPlannerColorLight::find_intersection(const Point_2 &p1, const Point_2 &p2, const Point_2 &p3, const Point_2 &p4)
{
    // Convert CGAL points to the exact kernel for computations
    Point_2 kp1(p1.x(), p1.y());
    Point_2 kp2(p2.x(), p2.y());
    Point_2 kp3(p3.x(), p3.y());
    Point_2 kp4(p4.x(), p4.y());

    Kernel::FT denominator = (kp1.x() - kp2.x()) * (kp3.y() - kp4.y()) - (kp1.y() - kp2.y()) * (kp3.x() - kp4.x());

    // Check if lines are parallel (denominator is zero)
    if (denominator == 0)
    {
        return 0; // No intersection (parallel lines)
    }

    Kernel::FT t = ((kp1.x() - kp3.x()) * (kp3.y() - kp4.y()) - (kp1.y() - kp3.y()) * (kp3.x() - kp4.x())) / denominator;
    Kernel::FT u = ((kp2.x() - kp1.x()) * (kp1.y() - kp3.y()) - (kp2.y() - kp1.y()) * (kp1.x() - kp3.x())) / denominator;

    // Check if the intersection point lies on both line segments
    if (t >= 0 && t <= 1)
    {
        return 1; // Intersection in the middle of first segment p1 and p2
    }
    else if (u >= 0 && u <= 1)
    {
        return 2; // Intersection in the middle of second segment p3 and p4
    }

    return 0; // No intersection
}

std::vector<Point_2> DTriangPlannerColorLight::offset_polyline(const std::vector<Point_2> &points, double offset_distance, bool is_initial_direction_left)
{
    std::vector<Point_2> offset_points;

    if (points.size() < 2)
    {
        return offset_points;
    }

    // Offset the first segment
    Point_2 new_p1, new_p2;
    for (int i = 0; i < points.size() - 1; i++)
    {  
        offset_segment(points[i], points[i + 1], offset_distance, is_initial_direction_left, new_p1, new_p2);

        if (i == 0)
        {
            offset_points.push_back(new_p1);
        }

        // Check condition for points 2:
        int condition = 0;
        if (i + 2 < points.size()) // Check bounds
        {
            condition = find_intersection(points[i + 1], points[i + 2], new_p1, new_p2);
        }        

        // if condition == 0, no intersection
        if (condition == 0)
        {
            offset_points.push_back(new_p2);
        }
        // if it intersection in the middle of first segment p1 and p2
        else if (condition == 1)
        {
            Point_2 p3;
            p3 = find_point_at_distance(new_p1, new_p2, offset_distance, true);
            offset_points.push_back(p3);
        }
        // if it intersection in the middle of second segment p3 and p4
        else if (condition == 2)
        {
            Point_2 p3;
            p3 = find_point_at_distance(new_p1, new_p2, offset_distance, false);
            offset_points.push_back(p3);
        }
    }

    return offset_points;
}
