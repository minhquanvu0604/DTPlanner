#include "d_triang_plot_color.hpp"


int main(int argc, char *argv[]) {
    QApplication app(argc, argv); 

    DTriangPlotColor plot = DTriangPlotColor();
    
    int config_number = 1;
    if (argc > 1)
        config_number = std::stoi(argv[1]);

    plot.set_config_num(config_number); 
    plot.plot();
    
    return app.exec();
}


DTriangPlotColor::DTriangPlotColor() : _config_num{1} {}


void DTriangPlotColor::plot(){
    
    read_cone_config_color();        
    set_cones_debug(_cones_local);
    update_order();


    // Triangulation edges
    std::vector<std::pair<Point_2, Point_2>> edges = get_edges_for_plotting();

    // Other path
    std::vector<std::vector<Point_2>> other_paths = get_other_paths();
    std::vector<std::pair<Point_2, Point_2>> other_paths_for_plotting;
    for (auto& other_path : other_paths){
        std::vector<std::pair<Point_2, Point_2>> path_for_plotting = get_path_for_plotting(other_path);
        other_paths_for_plotting.insert(other_paths_for_plotting.end(), path_for_plotting.begin(), path_for_plotting.end());
    }

    // Best path
    std::vector<Point_2> best_path = get_ultimate_path(); 
    std::vector<std::pair<Point_2, Point_2>> best_path_for_plotting = get_path_for_plotting(best_path);

    // Smooth path
    std::vector<std::pair<Point_2, Point_2>> smooth_path_for_plotting;
    // if (SMOOTH_PATH){
    //     std::vector<Point_2> smooth_best_path = catmull_rom_spline(best_path, 0.5);
    //     // std::cout << "smooth_best_path size: " << smooth_best_path.size() << std::endl;
    //     smooth_path_for_plotting = get_path_for_plotting(smooth_best_path); 
    // }

    // std::cout << "_local_pts_for_triang.size()xxxxxxxxxxxxxxxxxxxxxxxxxx" << _local_pts_for_triang.size() << std::endl;

    _widget = std::make_shared<PlotWidget>(_points_global, _local_pts_for_triang, edges, other_paths_for_plotting, best_path_for_plotting, smooth_path_for_plotting);    
    _widget->resize(800, 900); 
    _widget->show();


}

void DTriangPlotColor::set_config_num(int config_num){
    _config_num = config_num;
}



void DTriangPlotColor::read_cone_config_color(){
    
    if (PATH_MAP_COLOR.find(_config_num) == PATH_MAP_COLOR.end()) {
        throw std::runtime_error("No cones layout config number " + std::to_string(_config_num));
    }
    std::string yaml_path = PATH_MAP_COLOR.at(_config_num);

    // Read YAML file
    std::string base_path = "../../plotting/cones_layout/color/";  
    std::string full_path = base_path + yaml_path;
    YAML::Node config = YAML::LoadFile(full_path);
    const auto& points_node = config["points"];

    std::vector<DTCL::Cone> cones_global; 

    for (const auto& point_node : points_node) {
        double x = point_node["x"].as<double>();
        double y = point_node["y"].as<double>();
        int color = point_node["color"].as<int>();
        // _points_global.push_back(Point_2(x, y));
        cones_global.push_back(DTCL::Cone(Point_2(x,y),color));
    }

    double car_x = 0.0, car_y = 0.0, car_yaw = 0.0;
    if (config["car_pose"]) {
        auto car_pose = config["car_pose"];
        if (car_pose["position"]) {
            car_x = car_pose["position"]["x"].as<double>();
            car_y = car_pose["position"]["y"].as<double>();
        }
        if (car_pose["orientation"]) 
            car_yaw = car_pose["orientation"]["yaw"].as<double>();
    } else 
        std::cerr << "Car pose not found in YAML file." << std::endl;

    _local_pts_for_triang.clear();
    // Convert to car's frame
    for (auto& cone_global : cones_global){
        Point_2 p_local = transform_to_car_frame(cone_global.point,car_x,car_y,car_yaw);

        // Print out to write unit test
        std::cout << p_local.x() << " , " << p_local.y() << std::endl;
        
        // Map to trace back global coordinates
        _local_global_map[p_local] = cone_global.point;

        _local_pts_for_triang.push_back(p_local); // Use this to input dt.insert()
        _cones_local.push_back(DTCL::Cone(p_local,cone_global.color));
    }
}

void DTriangPlotColor::update_order(){
    // Get the correct order of -- GLOBAL POINTS -- 
    _points_global.clear(); 
    // _cones_global.clear();
    // _cones_local.clear();
    std::vector<DTCL::Cone> updated_local_cones;

    for (const Point_2& pt_local : _local_pts_for_triang) {

        Point_2 point_global = _local_global_map[pt_local];

        // _cones_global.push_back(cone_global);
        _points_global.push_back(point_global);

        // Rearring to the correct order of _cones_local
        for (DTCL::Cone cone : _cones_local){
            if (is_approx_equal(pt_local,cone.point,1e-3)){
                updated_local_cones.push_back(cone);
                continue;
            }
        }
    }
    _cones_local = updated_local_cones;
}


// std::vector<std::pair<Point_2, Point_2>> DTriangPlotColor::get_other_paths_for_plotting(std::vector<std::vector<Point_2>> paths){
//     std::vector<std::pair<Point_2, Point_2>> segments;
//     for (const auto& path : paths) {
//         for (size_t i = 0; i < path.size() - 1; ++i) {
//             segments.emplace_back(path[i], path[i + 1]);
//         }
//     }
//     return segments;
// }

std::vector<std::pair<Point_2, Point_2>> DTriangPlotColor::get_path_for_plotting(std::vector<Point_2> path){
    
    std::vector<std::pair<Point_2, Point_2>> edges;

    if (path.empty())
        return edges;
        // throw std::runtime_error("get_path_for_plotting: Path is empty");

    for (size_t i = 0; i < path.size() - 1; ++i) {
        edges.push_back(std::make_pair(path.at(i), path.at(i + 1)));
    }
    return edges;
}