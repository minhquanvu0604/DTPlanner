#ifndef D_TRIANG_PLOT_H
#define D_TRIANG_PLOT_H

#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>
#include <yaml-cpp/yaml.h>

#include "d_triang_planner.hpp"
#include "d_triang_Qt.hpp"


class DTriangPlot : public DTriangPlanner {

public:
    DTriangPlot();

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
    void update_global_points();

    /*
     * Get the edges for plotting
    */
    // std::vector<std::pair<Point_2, Point_2>> get_edges_for_plotting();

    std::vector<std::pair<Point_2, Point_2>> get_path_for_plotting(std::vector<Point_2> path);


private:

    int _config_num;
    bool _color_mode;

    const bool SMOOTH_PATH = true; 

    const std::unordered_map<int, std::string> PATH_MAP = {
        {1, "straight.yaml"},
        {2, "straight_with_stretch_p.yaml"},
        {3, "straight_missing_cone.yaml"}, // Fail
        {4, "slight_curve.yaml"},
        {5, "slight_curve_missing_cone1.yaml"}, // Fail
        {6, "slight_curve_missing_cone2.yaml"},
        {7, "slight_curve_missing_cone3.yaml"},
        {8, "near_steep_turn.yaml"},        
        {9, "multi_step1_1.yaml"}, // Eufs track
        {10, "multi_step1_3.yaml"},
        {11, "starting_position_sim.yaml"}
    };

    const std::unordered_map<int, std::string> PATH_MAP_COLOR = {
    };



    // COLORBLIND
    std::vector<Point_2> _points_global;
    // Create map for global and corresponding local point
    std::map<Point_2,Point_2> _local_global_map;


    // COLOR
    std::vector<DTCL::Cone> _cones_global;
    std::map<Point_2,DTCL::Cone> _local_global_cone_map;

    std::shared_ptr<PlotWidget> _widget;


};

#endif