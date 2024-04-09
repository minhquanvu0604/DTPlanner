#ifndef D_TRIANG_PLOT_COLOR_H
#define D_TRIANG_PLOT_COLOR_H

#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>
#include <yaml-cpp/yaml.h>

#include "d_triang_planner_color_light.hpp"
#include "d_triang_Qt.hpp"


class DTriangPlotColor : public DTriangPlannerColorLight {

public:
    DTriangPlotColor();

    /*
     * Main function to plot
    */
    void plot();

    void set_config_num(int config_num);

    /*
    Read config files to get global cones positions along with other values
    Convert global cones position to local car's frame

    Update member variable _points_local and _points_global

    In real code the car's pose is already in the origin
    
    
    */
    void read_cone_config();

    void read_cone_config_color();
    
    /*
     * After triangulation, the elements in local points are rearranged 
     * Update the global points to match with them
    */
    void update_order();

    /*
     * Get the edges for plotting
    */
    // std::vector<std::pair<Point_2, Point_2>> get_edges_for_plotting();

    std::vector<std::pair<Point_2, Point_2>> get_path_for_plotting(std::vector<Point_2> path);


private:

    int _config_num;
    bool SMOOTH_PATH = true; 


    const std::unordered_map<int, std::string> PATH_MAP_COLOR = {
        {1, "slight_curve_color.yaml"},
        {2, "near_steep_turn_color.yaml"},
        {3, "only_blue_color.yaml"},
        {4, "only_blue_2.yaml"},
        {5, "steep_turn_color.yaml"},
        {6, "error_2.yaml"},
        {7, "error_3.yaml"},
        {8, "error_4.yaml"},
        {9, "neighbor_not_face_handle_error.yaml"},
        {10, "vertices_not_connected_by_an_edge_error.yaml"},
        {11, "error_core_dump.yaml"}, // 3 yellow cones followed by 6 blue cones -> complicated
        {12, "only_blue_wrong_last_segment.yaml"},
        {13, "one_blue_cone.yaml"}

    };



    // // COLORBLIND
    
    // // Create map for global and corresponding local point
    // std::map<Point_2,Point_2> _local_global_map;

    std::vector<Point_2> _points_global;

    // std::vector<DTCL::Cone> _cones_global; // Set by yaml
    std::vector<DTCL::Cone> _cones_local; // To input to planner


    std::map<Point_2,Point_2> _local_global_map;

    std::shared_ptr<PlotWidget> _widget;


};
#endif