#ifndef D_TRIANG_PLANNER_COLOR_LIGHT_H
#define D_TRIANG_PLANNER_COLOR_LIGHT_H

#include <iostream>
#include <vector>
#include <memory>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>
#include <yaml-cpp/yaml.h>
#include <limits>
#include <cmath>
#include <queue>

#include "d_triang_types.hpp"

/**
Use light weight solution since we don't know the performance of computer vision
Don't apply reusing cones anymore, sacrifice more computation for path refining
TODO Assume no missing cones -> filter long edges
Filter out outlying cones

USAGE: 
    set_cones() -> getter

TO RESEARCH:
    Orange cones?

*/
class DTriangPlannerColorLight {

public:
    DTriangPlannerColorLight(){}

    /**
    Separate group of paths: 
        GROUP 1: Each node surrounded by 1 cone of either color (if exists would be closer to ther car then the other group)
        GROUP 2: Each node surrounded by 2 cones of both colors
    If no cones provide -> return false
    */
    bool set_cones(const std::vector<DTCL::Cone>& cones_local);

    bool set_cones_debug(const std::vector<DTCL::Cone>& cones_local);


    /*
    Needs improving to proper offset
    */
    std::vector<Point_2> process_group_1(const std::vector<DTCL::Cone>& group_1, int nearest_color);

    /*
    Doesn't choose next suitable point from car 
    */
    std::vector<Point_2> process_group_1_mtc(const std::vector<DTCL::Cone>& group_1, int nearest_color);
    
    std::vector<Point_2> process_group_2(const Point_2& starting_pt);

    // std::shared_ptr<DT::Node> get_last_node_path_group_1(const std::vector<Point_2>& path_group_1);
    // std::vector<Point_2> offset_line(const std::vector<Point_2>& line, bool offset_to_left);

    Point_2 find_lookahead_point();

    /**
    Perform triangulation
    */
    void triangulate(const std::vector<DTCL::Cone>& group_2);

    /**
    Return the edge to start with
        If group 1 is empty: return the Edge nearest to the car 
        Or else: last waypoint of group 1
    

    Return a copied edge to navigate inside the triangulation. 
    
    In CGAL, even though nearest_edge is a copy, it still represents a valid edge in the triangulation. 
    The copied edge retains the necessary information (face handle and index) to identify the specific edge 
    in the triangulation, allowing you to use it for further operations or navigation within the triangulation structure.
    */
    Edge get_first_edge(const Point_2& starting_pt);

    /**
    PUBLIC FOR TESTING ONLY
    Expand all the possible paths in a bread-first fashion
    */
    std::vector<std::vector<Point_2>> expand(const Edge& first_edge, const Point_2& starting_pt);

    std::vector<std::vector<Point_2>> expand_debug(const Edge& first_edge, const Point_2& starting_pt);


    /**
    PUBLIC FOR TESTING ONLY
    */
    std::vector<Edge> get_next_edges(Edge current_edge, Edge previous_edge);   




    // Testing function 
    DelaunayTriangulation* get_triangulation_ptr();
    Point_2 transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw);
    void print_all_possible_paths(std::vector<std::vector<Point_2>> paths);
    bool is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance); // Also used by test fixture



    /**
    Get the best path from all complete paths, which is meant to be the chosen path 
    for the car to follow
    */
    std::vector<Point_2> get_ultimate_path();


    std::vector<Point_2> get_best_path_group_2();

    /**
    Get other complete paths besides the best one
    */
    std::vector<std::vector<Point_2>> get_other_paths();


    /// Offsetting paths utilities for group 1
    double cross_product(const Point_2 &a, const Point_2 &b);
    void offset_segment(const Point_2 &p1, const Point_2 &p2, double offset_distance, bool direction, Point_2 &new_p1, Point_2 &new_p2);
    Point_2 find_point_at_distance(const Point_2& p1, const Point_2& p2, double distance, bool condition);
    int find_intersection(const Point_2 &p1, const Point_2 &p2, const Point_2 &p3, const Point_2 &p4);
    std::vector<Point_2> offset_polyline(const std::vector<Point_2> &points, double offset_distance, bool initial_direction);


    // ---- Printing utilities ---- //
    void print_edge_vertices(const Edge& edge);
    void print_path(const std::vector<Point_2>& path);
    void print_cones(const std::vector<DTCL::Cone>& cones);


protected:
    DelaunayTriangulation _dt;
    
    // Only for computing triangulation, not used for group 1 or path reusing
    // Set as member variable for easy retrieveing inside test
    std::vector<Point_2> _local_pts_for_triang; // REALLY NEEDED? PROB NOT

    std::vector<DTCL::Cone> _group_2;


    /**
    For visualisation
    */
    std::vector<std::pair<Point_2, Point_2>> get_edges_for_plotting();

    /*
    p1 is current node, p2 is next node
    */
    double compute_orientation(const Point_2& p1, const Point_2& p2);

private:
    std::pair<Point_2, Point_2> get_points_from_edge(const Edge& edge);

    /*
    Path evaluation process happens here
    */
    std::vector<Point_2> backtrack_path(const std::shared_ptr<DTCL::Node>& leaf_node);

    void choose_best_path(std::vector<std::vector<Point_2>>);


    // ---- Utilities ---- //

    bool are_edges_equal(const Edge& e1, const Edge& e2);

    // Normalize the angle difference to the range (-π, π)
    double normalize(double angle); // static?


protected: 
    const double PATH_PLANNING_RATE = 30;
    const double DT = 1 / PATH_PLANNING_RATE;
    double _last_angle = 0;


    const double TRACK_WIDTH = 3.0;
    const double LOOKAHEAD_DISTANCE = 3.0;
    const double CAR_ACCELERATION = 0.1;
    
    // All the possible paths found, each vector of Point_2 represents a path
    std::vector<std::vector<Point_2>> _paths_group_2;
    std::vector<Point_2> _best_path_group_2;

    std::vector<std::vector<Point_2>> _other_paths_group_2;

    std::vector<Point_2> _ultimate_path;
};

#endif