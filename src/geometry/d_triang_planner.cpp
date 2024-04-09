#include "d_triang_planner.hpp"
#include <cmath>


/*
 * Top level function that handles planning 
 *
 * Workflow:
 * Expand -> Preliminary condition -> Add to path -> Complete path
 * -> Repeat -> Evaluate paths 
*/
void DTriangPlanner::plan(){
    Edge first_edge = triangulate();
    expand(first_edge);
    // choose_best_path();
    choose_best_path_2();
    // std::cout << "exit plan()" << std::endl;
}


void DTriangPlanner::set_cones(const std::vector<Point_2>& points_local){
        
        _paths.clear();
        _paths_2.clear();
        // Don't have to clear _best_path and _best_path_2

        _points_local.clear();

        _points_local = points_local;


        // for (const auto& point : _points_local) {
        //     _pts_status.emplace_back(point, false);
        // }
    }

Edge DTriangPlanner::triangulate(){

    _dt.clear();

    // Triangulation computed here
    _dt.insert(_points_local.begin(), _points_local.end()); 

    // The two nearest points are ones with smallest x
    DelaunayTriangulation::Vertex_handle smallest, second_smallest;
    double smallest_x = std::numeric_limits<double>::max(), second_smallest_x = std::numeric_limits<double>::max();

    for (auto v = _dt.finite_vertices_begin(); v != _dt.finite_vertices_end(); ++v) {
        double x_value = CGAL::to_double(v->point().x());
        if (x_value < smallest_x) {
            second_smallest = smallest;
            second_smallest_x = smallest_x;
            smallest = v;
            smallest_x = x_value;
        } else if (x_value < second_smallest_x) {
            second_smallest = v;
            second_smallest_x = x_value;
        }
    }
            
    // From 2 nearest vertices, find the corresponding edge
    Edge nearest_edge;
    bool edge_found = false;

    Edge_circulator c = _dt.incident_edges(smallest), done(c);
    if (c != 0) {
        do {
            if ((c->first->vertex((c->second + 1) % 3) == second_smallest) || (c->first->vertex((c->second + 2) % 3) == second_smallest)) {
                nearest_edge = *c;
                edge_found = true;
                break;
            }
        } while (++c != done);
    }

    // Logic error checking
    if (!edge_found) 
        throw std::runtime_error("Vertices are not connected by an edge");

    if (_dt.is_infinite(nearest_edge))
        throw std::runtime_error("The nearest edge is infinite");

    // std::cout << "The edge:";
    // print_edge_vertices(nearest_edge);

    return nearest_edge;
}

/*
 * 
 * NOTE: need to confirm that start_edge is not infinite 
 * 
 * POTENTIAL FEATURES
 *      Save rejected paths -> prevent the case where the good path is rejected by the last segment
*/
void DTriangPlanner::expand(Edge start_edge) {

    // A queue to store the instantaneous traversing progress 
    // Each has 2 edges represents a state of traverse progress
    // They represent the previous and current edge respectively
    std::deque<DT::TraverseState> traverse_progress;

    std::shared_ptr<DT::Node> car_node_ptr = std::make_shared<DT::Node>(Pose(0,0,0));

    std::pair<Point_2, Point_2> two_pts_first_node = get_points_from_edge(start_edge);
    Point_2 first_node_midpoint = CGAL::midpoint(two_pts_first_node.first, two_pts_first_node.second);

    double first_node_angle = compute_orientation(car_node_ptr->pose.position, first_node_midpoint);
    first_node_angle = abs(normalize(first_node_angle));
    
    std::shared_ptr<DT::Node> first_node_ptr = std::make_shared<DT::Node>(Pose(first_node_midpoint, first_node_angle));
    first_node_ptr->cones = {two_pts_first_node.first, two_pts_first_node.second};

    // DEBUGGING
    std::cout << "FIRST NODE: " << std::endl;
    print_edge_vertices(start_edge);
    std::cout << "Midpoint: " << first_node_midpoint << std::endl;
    std::cout << "Previous point: " << car_node_ptr->pose.position << std::endl;
    std::cout << "Angle diff: " << first_node_angle << std::endl; 
    std::cout << "=====================================" << std::endl; 

    int passed = 0; // Number of next nodes that pass the 'preliminary condtion'
    int num_node_checked = 0; // Total number of nodes checked
    
    DT::TraverseState initial_state;
    initial_state.node_ptr = first_node_ptr;
    initial_state.current_edge = start_edge;
    initial_state.previous_edge = Edge();

    traverse_progress.push_back(initial_state);

    while (!traverse_progress.empty()) {

        DT::TraverseState current_state = traverse_progress.front();
        traverse_progress.pop_front();
        
        // Expand on the last state
        std::vector<Edge> next_edges = get_next_edges(current_state.current_edge, current_state.previous_edge);
        std::shared_ptr<DT::Node> last_node_ptr = current_state.node_ptr;
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
                _paths.push_back(path);
                std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path_2 = backtrack_path_2(last_node_ptr);
                _paths_2.push_back(path_2);
                
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
                std::cout << "REJECTED" << std::endl;
                std::cout << "=====================================" << std::endl;
                continue;
            }

            std::cout << "PASSED" << std::endl;
            passed++;

            // Create new node, the node is created here and only here
            std::shared_ptr<DT::Node> new_node_ptr = std::make_shared<DT::Node>(Pose(midpoint, midpoint_angle));
            // Register this node
            // Connect this node to last node
            new_node_ptr->parent_ptr = last_node_ptr;
            last_node_ptr->children_ptrs.push_back(new_node_ptr);
            // Add cone information of this node
            new_node_ptr->cones = {two_pts.first, two_pts.second};


            // Otherwise add to new state
            DT::TraverseState new_state(new_node_ptr, next_new_edge, current_new_edge);
            traverse_progress.push_back(new_state);                

            std::cout << "=====================================" << std::endl; 
        }    
    }
    std::cout << "FINISHED GENERATING PATHS" << std::endl;
    std::cout << "Passed: " << passed << std::endl;
    std::cout << "Total number of nodes checked: " << num_node_checked << std::endl;
    print_all_possible_paths();
    // std::cout << "exit expand()" << std::endl;
}


std::vector<std::vector<Point_2>> DTriangPlanner::get_paths(){
    return _paths;
}
std::vector<std::vector<std::pair<Point_2, std::array<Point_2, 2>>>> DTriangPlanner::get_paths_2(){
    return _paths_2;
}


std::vector<Point_2> DTriangPlanner::get_best_path(){
    return _best_path;
}
std::vector<std::pair<Point_2, std::array<Point_2, 2>>> DTriangPlanner::get_best_path_2(){
    return _best_path_2;
}


std::vector<std::vector<Point_2>> DTriangPlanner::get_other_paths(){
    return _other_paths;
}


DelaunayTriangulation* DTriangPlanner::get_triangulation_ptr() {
    return &_dt;
}


std::vector<Point_2> DTriangPlanner::get_all_vertices() {
    std::vector<Point_2> vertices;
    for (auto vit = _dt.finite_vertices_begin(); vit != _dt.finite_vertices_end(); ++vit) {
        vertices.push_back(vit->point());
    }
    return vertices;
}

std::vector<std::pair<Point_2, Point_2>> DTriangPlanner::get_edges_for_plotting(){
    // Get the edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    for (auto it = _dt.finite_edges_begin(); it != _dt.finite_edges_end(); ++it) {
        Kernel::Segment_2 segment = _dt.segment(it);
        edges.emplace_back(segment.start(), segment.end());
    }
    return edges;
}

/////////////////// PRIVATE FUNCTIONS ///////////////////////////////////////

/*
 * Get next 2 edges from the current edge, moving to the next face
 * Returns empty vector means the path meets the dead end
 * 
 * If the edge is a starting position of the path, previous_edge will be created by the defautl contructor
 * 
 * Both edge must refer to the same face (current_edge.first == previous_edge.first)
 * 
 * Returns 3 edges, the first one is the current_edge with respect to new _face
 * The two other edges are others of the new_face
*/
std::vector<Edge> DTriangPlanner::get_next_edges(Edge current_edge, Edge previous_edge) {
    
    std::vector<Edge> next_edges;
    Edge edge0;
    Edge edge1;
    Edge edge2;

    // The case of first edge
    if (previous_edge.first == DelaunayTriangulation::Face_handle()) {
        std::cout << "FIRST EDGE" << std::endl;

        DelaunayTriangulation::Face_handle new_face = current_edge.first;

        // There may be circumstances where the current_edge is not inifinite but 
        // current_edge.first is still inifinite 
        if (_dt.is_infinite(new_face)){
            new_face = current_edge.first->neighbor(current_edge.second);
            // std::cout << "new_face:" << std::endl;
            // print_face_vertices(new_face);

            // In this case we have to find the correct edge index
            int new_index = -1;
            for (int i = 0; i < 3; ++i) {
                Edge possible_edge(new_face, i);
                if (are_edges_equal(possible_edge, current_edge)) {
                    new_index = i;
                    break;
                }
            }
            if (new_index == -1) {
                throw std::runtime_error("Could not find the edge in the new face.");
            }

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


void DTriangPlanner::choose_best_path(){
    int max_size = 0;
    int index_of_longest = -1;

    for (int i = 0; i < _paths.size(); ++i) {
        if (_paths[i].size() > max_size) {
            max_size = _paths[i].size();
            index_of_longest = i;
        }
    }

    _best_path = _paths[index_of_longest];
    
    Point_2 car_pt = Point_2(0,0);
    _best_path.insert(_best_path.begin(), car_pt); // Add the car point only to the best path


    for (int i = 0; i < _paths.size(); ++i) {
        if (i != index_of_longest) {
            _other_paths.push_back(_paths[i]);
        }
    }
    
}

void DTriangPlanner::choose_best_path_2(){
    
    // For _best_path_2
    int max_size = 0;
    int index_of_longest = -1;

    for (int i = 0; i < _paths_2.size(); ++i) {
        if (_paths_2[i].size() > max_size) {
            max_size = _paths_2[i].size();
            index_of_longest = i;
        }
    }
    // Important to have this 
    // If index = -1, C++ exception with description "std::bad_alloc" thrown in the test body
    if (index_of_longest == -1) 
        throw std::runtime_error("_paths is empty");

    _best_path_2 = _paths_2[index_of_longest];


    // For _best_path
    max_size = 0;
    index_of_longest = -1;

    for (int i = 0; i < _paths.size(); ++i) {
        if (_paths[i].size() > max_size) {
            max_size = _paths[i].size();
            index_of_longest = i;
        }
    }
    if (index_of_longest == -1)
        throw std::runtime_error("_paths is empty");
    _best_path = _paths[index_of_longest];

    
    std::pair<Point_2, std::array<Point_2, 2>> car_node;
    car_node.first = Point_2(0,0);
    _best_path_2.insert(_best_path_2.begin(), car_node); // Add the car point only to the best path

    // For other paths, dont' have to save the 2 cones for each nodes 
    for (int i = 0; i < _paths.size(); ++i) {
        if (i != index_of_longest) {
            _other_paths.push_back(_paths[i]);
        }
    }
    
}


// Point_2 DTriangPlanner::get_midpoint_of_edge(const Edge& edge) {
//     Point_2 p1 = edge.first->vertex((edge.second + 1) % 3)->point();
//     Point_2 p2 = edge.first->vertex((edge.second + 2) % 3)->point();
//     return CGAL::midpoint(p1, p2);
// }

std::pair<Point_2, Point_2> DTriangPlanner::get_points_from_edge(const Edge& edge) {
    auto face = edge.first;
    int i = edge.second;

    Point_2 p1 = face->vertex((i + 1) % 3)->point(); // Next vertex in the face
    Point_2 p2 = face->vertex((i + 2) % 3)->point(); // Next vertex after p1 in the face

    return std::make_pair(p1, p2);
}

double DTriangPlanner::compute_orientation(const Point_2& p1, const Point_2& p2) {
    double deltaX = CGAL::to_double(p2.x()) - CGAL::to_double(p1.x());
    double deltaY = CGAL::to_double(p2.y()) - CGAL::to_double(p1.y());

    // std::cout << "deltaX: " << deltaX << std::endl;
    // std::cout << "deltaY: " << deltaY << std::endl;
    // std::cout << "std::atan2(deltaY, deltaX): " << std::atan2(deltaY, deltaX) << std::endl;

    return std::atan2(deltaY, deltaX); // Returns angle in radians
}


double DTriangPlanner::normalize(double angle){

    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }    
    return angle;
}


std::vector<Point_2> DTriangPlanner::backtrack_path(const std::shared_ptr<DT::Node>& leaf_node) {
    std::vector<Point_2> path;
    std::shared_ptr<DT::Node> current_node = leaf_node;


    while (current_node) {
        path.push_back(current_node->pose.position);
        current_node = current_node->parent_ptr;
    }

    // Reverse the path to get it from root to leaf
    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<std::pair<Point_2, std::array<Point_2, 2>>> DTriangPlanner::backtrack_path_2(const std::shared_ptr<DT::Node>& leaf_node) {
    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path;
    
    std::shared_ptr<DT::Node> current_node = leaf_node;

    while (current_node) {

        Point_2 point = current_node->pose.position;
        std::array<Point_2, 2> cones = current_node->cones;
        std::pair<Point_2, std::array<Point_2, 2>> node = std::make_pair(point, cones);
        path.push_back(node);

        current_node = current_node->parent_ptr;
    }

    // Reverse the path to get it from root to leaf
    std::reverse(path.begin(), path.end());
    return path;
}


bool DTriangPlanner::are_edges_equal(const Edge& e1, const Edge& e2) {
    double tolerance = 1e-3;
    auto seg1 = _dt.segment(e1);
    auto seg2 = _dt.segment(e2);

    return (is_approx_equal(seg1.source(), seg2.source(), tolerance) && is_approx_equal(seg1.target(), seg2.target(), tolerance)) ||
           (is_approx_equal(seg1.source(), seg2.target(), tolerance) && is_approx_equal(seg1.target(), seg2.source(), tolerance));
}

bool DTriangPlanner::is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance) {
    return CGAL::squared_distance(p1, p2) < tolerance * tolerance;
}

std::vector<Point_2> DTriangPlanner::catmull_rom_spline(const std::vector<Point_2>& waypoints, double t) {
    std::vector<Point_2> smooth_path;

    if (waypoints.size() < 4) {
        return waypoints;
    }

    smooth_path.push_back(waypoints.front());

    for (size_t i = 1; i < waypoints.size() - 2; ++i) {
        for (double t_iter = 0; t_iter < 1; t_iter += t) {
            double t2 = t_iter * t_iter;
            double t3 = t2 * t_iter;

            // Catmull-Rom spline equation coefficients
            double p0 = (-t3 + 2*t2 - t_iter) / 2;
            double p1 = (3*t3 - 5*t2 + 2) / 2;
            double p2 = (-3*t3 + 4*t2 + t_iter) / 2;
            double p3 = (t3 - t2) / 2;

            // Calculating point coordinates
            double x = CGAL::to_double(waypoints[i - 1].x()) * p0 + CGAL::to_double(waypoints[i].x()) * p1 + CGAL::to_double(waypoints[i + 1].x()) * p2 + CGAL::to_double(waypoints[i + 2].x()) * p3;
            double y = CGAL::to_double(waypoints[i - 1].y()) * p0 + CGAL::to_double(waypoints[i].y()) * p1 + CGAL::to_double(waypoints[i + 1].y()) * p2 + CGAL::to_double(waypoints[i + 2].y()) * p3;

            // Constructing a new point
            Point_2 point(x, y);

            smooth_path.push_back(point);
        }
    }

    smooth_path.push_back(waypoints.back());

    return smooth_path;
}

std::vector<Point_2> DTriangPlanner::bezier_curve(const std::vector<Point_2>& control_points, int num_points) {
    std::vector<Point_2> bezier_curve;
    int n = control_points.size() - 1; // degree of the curve

    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        std::vector<Point_2> temp = control_points;

        // De Casteljau's Algorithm
        for (int k = 1; k <= n; ++k) {
            for (int j = 0; j <= n - k; ++j) {
                double x = (1 - t) * CGAL::to_double(temp[j].x()) + t * CGAL::to_double(temp[j + 1].x());
                double y = (1 - t) * CGAL::to_double(temp[j].y()) + t * CGAL::to_double(temp[j + 1].y());
                temp[j] = Point_2(x, y);
            }
        }

        bezier_curve.push_back(temp[0]);
    }

    return bezier_curve;
}

std::vector<Point_2> DTriangPlanner::calculate_control_points(const std::vector<Point_2>& path, double curvature_parameter) {
    std::vector<Point_2> control_points;

    control_points.push_back(path.front());
    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Get two consecutive points
        Point_2 p1 = path[i];
        Point_2 p2 = path[i + 1];

        // Calculate mid-point
        double mid_x = (CGAL::to_double(p1.x()) + CGAL::to_double(p2.x())) / 2;
        double mid_y = (CGAL::to_double(p1.y()) + CGAL::to_double(p2.y())) / 2;

        // Calculate a perpendicular vector
        double dx = CGAL::to_double(p2.x()) - CGAL::to_double(p1.x());
        double dy = CGAL::to_double(p2.y()) - CGAL::to_double(p1.y());
        double length = sqrt(dx * dx + dy * dy);
        double perp_dx = -dy / length;
        double perp_dy = dx / length;

        // Calculate control points using the curvature parameter
        Point_2 control_point1(mid_x + perp_dx * curvature_parameter, mid_y + perp_dy * curvature_parameter);
        Point_2 control_point2(mid_x - perp_dx * curvature_parameter, mid_y - perp_dy * curvature_parameter);

        control_points.push_back(control_point1);
        control_points.push_back(control_point2);
    }
    control_points.push_back(path.back());


    return control_points;
}

std::vector<Point_2> DTriangPlanner::smooth_path(const std::vector<Point_2>& path) {
    const double curvature_parameter = 2;
    std::vector<Point_2> control_points = calculate_control_points(path, curvature_parameter);
    int num_curve_points = path.size() - 1;
    return bezier_curve(control_points, num_curve_points);
}


//////////////////////////////// TESTING FUNCTION ///////////////////////////////////////



void DTriangPlanner::print_all_possible_paths() {
    std::cout << "Number of paths: " << _paths.size() << std::endl;

    for (const auto& path : _paths) {
        std::cout << "Path: ";
        for (const auto& point : path) {
            std::cout << "(" << point.x() << ", " << point.y() << ") ";
        }
        std::cout << std::endl;
    }
}

Point_2 DTriangPlanner::transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw) {

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}



//////

void DTriangPlanner::print_face_vertices(DelaunayTriangulation::Face_handle face) {
    if (!face->is_valid()) {
        std::cerr << "Invalid face handle." << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i) {
        Point_2 vertex = face->vertex(i)->point();
        std::cout << "Vertex " << i << ": (" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
    }
}

void DTriangPlanner::print_edge_vertices(const Edge& edge) {
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

void DTriangPlanner::print_path(const std::vector<Point_2>& path) {
    for (const Point_2& point : path) {
        std::cout << "Point (" << point << ")  ";
    }
    std::cout << std::endl;
}

void DTriangPlanner::print_path_2(const std::vector<std::pair<Point_2, std::array<Point_2, 2>>>& path) {
    std::cout << "Print anything?" << std::endl;
    for (const auto& waypoint : path) {
        const Point_2& position = waypoint.first;
        const std::array<Point_2, 2>& cones = waypoint.second;

        std::cout << "Waypoint: (" << CGAL::to_double(position.x()) << ", " << CGAL::to_double(position.y()) << ")\n";
        std::cout << "  Cone 1: (" << CGAL::to_double(cones[0].x()) << ", " << CGAL::to_double(cones[0].y()) << ")\n";
        std::cout << "  Cone 2: (" << CGAL::to_double(cones[1].x()) << ", " << CGAL::to_double(cones[1].y()) << ")\n";
    }
}
