#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>
#include <yaml-cpp/yaml.h>
#include <limits>
#include <cmath>

#include "../plotting/include/d_triang_Qt.hpp"
#include "d_triang_types.hpp"

// typedef DT::Node Node;
// typedef DT::Pose Pose;


Point_2 transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw) {

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}

/*
 * STEPS TO CREATE THE PATH 
*/
int main(int argc, char *argv[]) {

    // ---------------------------- STEP 1: INPUT THE POINTS---------------------------- //

    // Generate a set of random points
    std::vector<Point_2> points_global; // Keep the -- GLOBAL POINTS -- for representation or localisation, not needed outside of test suit at the moment
    std::vector<Point_2> points_local;

    int number = 1;
    if (argc > 1)
        number = std::stoi(argv[1]);
    std::unordered_map<int, std::string> path_map = {
        {1, "straight.yaml"},
        {2, "straight_with_stretch_p.yaml"},
        {3, "straight_missing_cone.yaml"},
        {4, "slight_curve.yaml"}
    };
    if (path_map.find(number) == path_map.end()) {
        std::cerr << "No cones layout number " << number << std::endl;
        return 1;
    }
    std::string yaml_path = path_map[number];

    // Read YAML file
    std::string base_path = "../cones_layout/";  
    std::string full_path = base_path + yaml_path;
    YAML::Node config = YAML::LoadFile(full_path);
    const auto& points_node = config["points"];
    for (const auto& point_node : points_node) {
        double x = point_node["x"].as<double>();
        double y = point_node["y"].as<double>();
        points_global.push_back(Point_2(x, y));
    }

    //-----------Qt -----------Qt -----------Qt -----------Qt -----------//
    // In real code the car's pose is in the origin
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

    // Create map for global and corresponding local point
    std::map<Point_2,Point_2> local_global_map;
    
    // Convert to car's frame
    for (Point_2 point_global : points_global){
        Point_2 p_local = transform_to_car_frame(point_global,car_x,car_y,car_yaw);
        local_global_map[p_local] = point_global;
        points_local.push_back(p_local);
    }
    //-----------Qt -----------Qt -----------Qt -----------Qt -----------//

    // Compute Delaunay Triangulation
    DelaunayTriangulation dt;
    dt.insert(points_local.begin(), points_local.end()); // Triangulation computed here

    // Get the correct order of -- GLOBAL POINTS -- 
    points_global.clear(); 
    for (const auto& point_local : points_local) {
        Point_2 point_global = local_global_map[point_local];
        points_global.push_back(point_global);
    }

    //-----------Qt -----------Qt -----------Qt -----------Qt -----------//
    // Get the edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    for (auto it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        Kernel::Segment_2 segment = dt.segment(it);
        edges.emplace_back(segment.start(), segment.end());
    }
    //-----------Qt -----------Qt -----------Qt -----------Qt -----------//
 
    // ---------------------------- STEP 2: FILTER EDGES BY LENGHT---------------------------- //
    // This is not good with missing cones 

    // // Filter edges of valid length
    // const double max_length = 14;
    // const double min_length = 7;
    // const double max_length_sq = std::pow(max_length,2);
    // const double min_length_sq = std::pow(min_length,2);

    // // Test print edge length 
    // for (auto it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    //     Kernel::Segment_2 segment = dt.segment(it);
    //     std::cout << std::pow(CGAL::to_double(segment.squared_length()),0.5) << std::endl;
    // }
    
    // // Core components so don't use std::vector<std::pair<Point_2, Point_2>> edges
    // std::vector<std::pair<Point_2, Point_2>> valid_edges;
    // for (auto it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    //     Kernel::Segment_2 segment = dt.segment(it);
    //     if (segment.squared_length() < max_length_sq && segment.squared_length() > min_length_sq){
    //         valid_edges.push_back(std::make_pair(segment.start(), segment.end()));
    //         // std::cout << "Take: "<< std::pow(CGAL::to_double(segment.squared_length()),0.5) << std::endl;
    //     }
    // }

    //-----Qt -----------Qt -----------Qt -----------Qt -----------//
    //-----------Qt -----------Qt -----------Qt -----------Qt -----------//
    QApplication app(argc, argv);                                                   
    PlotWidget widget(points_global, points_local, edges);
    widget.resize(800, 900); 
    widget.show();
    return app.exec();
    //-----------Qt -----------Qt -----------Qt -----------Qt -----------//
    //-----Qt -----------Qt -----------Qt -----------Qt -----------//

    // ---------------------------- STEP 2: FIND STARTING NODE ----------------------------- //

    // Node car_node(car_x, car_y, car_yaw);
    // Get the first node, which is the midpoint closest to the car
    // UNNECESSARY: THE POINTS ARE IN LOCAL FRAME, SO THE NEAREST POINT IN FRONT OF THE CAR IS THE ONE
    // WITH SMALLEST X

    // // Find the closest vertex
    // Point_2 car_position(car_x, car_y); 
    // DelaunayTriangulation::Vertex_handle nearest_vertex = dt.nearest_vertex(car_position);

    // // Find the closest midpoint
    // Point_2 closest_midpoint;
    // DelaunayTriangulation::Edge closest_edge;
    // double min_dist = std::numeric_limits<double>::max();

    // DelaunayTriangulation::Edge_circulator c = dt.incident_edges(nearest_vertex), done(c);
    // if (c != 0) {
    //     do {
    //         auto edge = *c;
    //         auto face = edge.first;
    //         int i = edge.second;

    //         Point_2 p1 = face->vertex((i+1)%3)->point();
    //         Point_2 p2 = face->vertex((i+2)%3)->point();
    //         Point_2 mid = midpoint(p1, p2);
            
    //         double dist = CGAL::to_double(CGAL::squared_distance(car_position, mid));

    //         if (dist < min_dist) {
    //             min_dist = dist;
    //             closest_midpoint = mid;
    //             closest_edge = edge;
    //         }
    //     } while (++c != done);
    // } // Found closest midpoint and the corresponding edge

    // The two nearest points are ones with smallest x
    DelaunayTriangulation::Vertex_handle smallest, second_smallest;
    double smallest_x = std::numeric_limits<double>::max(), second_smallest_x = std::numeric_limits<double>::max();

    for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); ++v) {
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
    DelaunayTriangulation::Edge nearest_edge;    
    DelaunayTriangulation::Edge_circulator c = dt.incident_edges(smallest), done(c);
    if (c != 0) {
        do {
            if ((c->first->vertex((c->second + 1) % 3) == second_smallest) || (c->first->vertex((c->second + 2) % 3) == second_smallest)) {
                nearest_edge = *c;
                break;
            }
        } while (++c != done);
    }
    throw std::runtime_error("Vertices are not connected by an edge.");



    // ---------------------------- STEP 3: CONSTRUCT THE TREE ----------------------------- //

    std::vector<std::shared_ptr<Node>> node_list; // LIST OF NODES HERE
    // node_list.push_back();





    // // Compute midpoints of internal edges and store in a vector
    // std::vector<Point_2> mid_points;
    // // Finite edges are those edges that are not part of the unbounded face of the triangulation.
    // for (auto edge = dt.finite_edges_begin(); edge != dt.finite_edges_end(); ++edge) { 
        
    //     // Check if the face containing the edge is infinite
    //     // In CGAL, triangulations can have infinite faces, especially when dealing with Delaunay triangulations 
    //     // This check ensures that only internal edges (edges of finite faces) are considered.
    //     if (!dt.is_infinite(edge->first)) {
    //         // Compute the midpoint of the edge and store it in the vector
    //         // The edge->second gives the index of the vertex opposite the edge
    //         // (edge->second + 1) % 3 and (edge->second + 2) % 3 give the indices of the vertices forming the edge.
    //         Point_2 midpoint = CGAL::midpoint(edge->first->vertex((edge->second + 1) % 3)->point(),
    //                               edge->first->vertex((edge->second + 2) % 3)->point());
    //         mid_points.push_back(midpoint);
    //     }
    // }

    // // Output the midpoints
    // std::cout << "Midpoints of internal edges:" << std::endl;
    // for (const auto& midpoint : midpoints) {
    //     std::cout << midpoint << std::endl;
    // }

    return 0;
}
