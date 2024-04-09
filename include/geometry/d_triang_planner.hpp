#ifndef D_TRIANG_PLANNER_H
#define D_TRIANG_PLANNER_H

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
@brief Base class for path planners

CONTRACT:
    INPUT : A list of cones to be used for planning at that instance
    OUTPUT : A path
        ---- Find a good data structure for them ----
    BEHAVIOUR : ....

USAGE:
    set_cones() -> plan() -> get_best_path()/other getters

ANALYSIS:
    Compromise the computer vision section
        Missing cones
        Incorrect color

    Loss function : based on compromisation of vision 
        Color
        Cone lining up in a reasonable curve
        Path is of reasonable curve
        Distance between consecutive cones
        Have cones on both side

TO RESEARCH:
    How to refine the path certainty other time
    Metrics for performance evaluation
    The path planner takes input from SLAM, but still stores meaningful
        data itself regarding past path
    ***Take advantage of color of the cones

TO EXPLORE:
    Efficient Data Handling: Ensure your path planner can efficiently process incoming data in real-time. This might involve optimizing data structures for faster access and updates.
        Data Structure & Algorithm: the CGAL lib may have high overhead since it's optimised for large set of poitns not repeated calculation with new set of points every time stamp
        Caching and Reuse: Cache previously calculated paths or path segments that are likely to be reused. When a similar planning scenario occurs, you can quickly retrieve and adapt these paths instead of starting from scratch.
            Matching Algorithm: Implement an algorithm to match the current cones' positions with the translated positions from the previous cycle. This can be done using nearest neighbor matching, where each cone in the current cycle is matched with the closest translated cone from the previous cycle.
                Thresholding: Set a maximum expected deviation from the average or median value. If a reading exceeds this threshold, it's considered an outlier and rejected.
                Running Average: Maintain a running average of the sensor readings. If a new reading is significantly different from this average, it can be discarded or given less weight.
                Median Filter: Use a median filter, which replaces each entry with the median of neighboring entries. This method is effective in removing "spike" noise.
        Parallel Data Processing: Utilize parallel computing techniques to process data concurrently, leveraging multi-core processors.
        Profile and Benchmark: Regularly profile the data handling process to identify bottlenecks and optimize them.

    Continuous Updating Mechanism: Your algorithm should be able to update its path planning based on new data without restarting from scratch. This could be achieved through incremental updates to the existing path or quick re-planning.
        Incremental Updates: Implement the ability to modify the existing path based on new data. For instance, if a new obstacle is detected, only the path segments affected by this obstacle should be recalculated.
        Maintaining a Balance: Strive for a balance between adapting to new data and sticking to the original plan. Constantly changing the path can lead to inefficiency and erratic behavior.
        Feedback Loops (Adaptive): Incorporate feedback mechanisms to continuously assess the validity of the path and make adjustments as needed
    
    Predictive Analysis: Use predictive models to anticipate future changes in the environment and adjust the path accordingly.
    
    Concurrency and Threading: Consider using multi-threading or asynchronous programming to handle data processing, path computation, and execution in parallel.

NOTE:
    The path always stems from the car body

EXPERIMENTAL INSIGHTS:
    The path with the highest number of Point_2 is not always the correct path (esp. in the case of missing cones) 
    The path that 'unlocks' the most cones is not alwasy the correct path (esp. in the case of missing cones)

 */
class DTriangPlanner {

public:
    DTriangPlanner(){}

    void set_cones(const std::vector<Point_2>& points_local);
    
    /*
    Main function to perform planning
    */
    void plan();

    /*
    Perform triangulation and return the nearest edge to start with
    Return a copied edge to navigate inside the triangulation. 
    
    In CGAL, even though nearest_edge is a copy, it still represents a valid edge in the triangulation. 
    The copied edge retains the necessary information (face handle and index) to identify the specific edge 
    in the triangulation, allowing you to use it for further operations or navigation within the triangulation structure.
    */
    Edge triangulate();


    /*
    PUBLIC FOR TESTING ONLY
    Expand all the possible paths in a bread-first fashion
    */
    void expand(Edge start);

    /*
    PUBLIC FOR TESTING ONLY
    */
    std::vector<Edge> get_next_edges(Edge current_edge, Edge previous_edge);   

    /*
    Catmull-Rom splines pass through each control point (unlike Bézier curves)

    Not very good so optional at the moment
    */
    std::vector<Point_2> catmull_rom_spline(const std::vector<Point_2>& waypoints, double t);


    // Not working yet
    std::vector<Point_2> bezier_curve(const std::vector<Point_2>& control_points, int num_points);
    std::vector<Point_2> calculate_control_points(const std::vector<Point_2>& path, double curvature_parameter);
    std::vector<Point_2> smooth_path(const std::vector<Point_2>& path);


    /*
    Get a list of complete paths
    */    
    std::vector<std::vector<Point_2>> get_paths();
    /*
    Used to save nodes along with their 2 cones
    */ 
    std::vector<std::vector<std::pair<Point_2, std::array<Point_2, 2>>>> get_paths_2();

    /*
    Get the best path from all complete paths, which is meant to be the chosen path 
    for the car to follow
    */
    std::vector<Point_2> get_best_path();
    /*
    Used to save nodes along with their 2 cones
    */ 
    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> get_best_path_2();

    /*
    Get other complete paths besides the best one
    */
    std::vector<std::vector<Point_2>> get_other_paths();

    /*
    For cone matching algorithm
    */
    std::vector<Point_2> get_all_vertices();

    /*
    For visualisation
    */
    std::vector<std::pair<Point_2, Point_2>> get_edges_for_plotting();


    // Testing function 
    DelaunayTriangulation* get_triangulation_ptr();
    Point_2 transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw);
    void print_all_possible_paths();
    bool is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance); // Also used by test fixture

    // Printing utilities
    void print_face_vertices(DelaunayTriangulation::Face_handle face);
    void print_edge_vertices(const Edge& edge);
    void print_path(const std::vector<Point_2>& path);
    void print_path_2(const std::vector<std::pair<Point_2, std::array<Point_2, 2>>>& path);


protected:
    DelaunayTriangulation _dt;
    std::vector<Point_2> _points_local;

private:    
    // All the possible paths found, each vector of Point_2 represents a path
    //
    // Just node location
    std::vector<std::vector<Point_2>> _paths;
    std::vector<Point_2> _best_path;
    //
    // Nodes along with their 2 cones
    std::vector<std::vector<std::pair<Point_2, std::array<Point_2, 2>>>> _paths_2;
    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> _best_path_2;


    
    std::vector<std::vector<Point_2>> _other_paths;


private:

    void choose_best_path();
    void choose_best_path_2();

    // Point_2 get_midpoint_of_edge(const Edge& edge);
    std::pair<Point_2, Point_2> get_points_from_edge(const Edge& edge);


    /*
    p1 is current node, p2 is next node
    */
    double compute_orientation(const Point_2& p1, const Point_2& p2);

    /*
    Path evaluation process happens here
    */
    std::vector<Point_2> backtrack_path(const std::shared_ptr<DT::Node>& leaf_node);

    /*
    Used to save nodes along with their 2 cones
    */ 
    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> backtrack_path_2(const std::shared_ptr<DT::Node>& leaf_node);

    // Normalize the angle difference to the range (-π, π)
    double normalize(double angle); // static?


    bool are_edges_equal(const Edge& e1, const Edge& e2);


private:

    // // Use this to count how many cones a path has gone between
    // std::vector<std::pair<Point_2, bool>> _pts_status;

    // The corresponding 
};



#endif