#ifndef REALTIME_TEST_FIXTURE_H
#define REALTIME_TEST_FIXTURE_H

#include <gtest/gtest.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <yaml-cpp/yaml.h>


#include "../../include/realtime/d_triang_realtime.hpp"


class DTRealTimeTestFixture : public ::testing::Test {
protected:

    DTRealTime planner;

    // virtual void SetUp() {

    //     std::vector<Point_2> points_local = get_cone_layout("near_steep_turn.yaml");

    //     // // Print out cone layout
    //     // for(auto p : points_local){
    //     //     std::cout << p << std::endl;
    //     // }
    //     // std::cout << std::endl;
    // }


    std::vector<std::vector<Point_2>> get_cone_layout_steps(std::string yaml_path){
        
        std::vector<std::vector<Point_2>> points_local_steps;
        std::vector<Point_2> points_global;

        std::string base_path = "../test/realtime/";  
        std::string full_path = base_path + yaml_path;
        YAML::Node config = YAML::LoadFile(full_path);
        const auto& points_node = config["points"];
        for (const auto& point_node : points_node) {
            double x = point_node["x"].as<double>();
            double y = point_node["y"].as<double>();
            points_global.push_back(Point_2(x, y));
        }

        std::vector<Point_2> points_local_step1 = get_cone_layout_step(config, "car_pose_step1", points_global);
        std::vector<Point_2> points_local_step2 = get_cone_layout_step(config, "car_pose_step2", points_global);
        std::vector<Point_2> points_local_step3 = get_cone_layout_step(config, "car_pose_step3", points_global);

        points_local_steps.push_back(points_local_step1);
        points_local_steps.push_back(points_local_step2);
        points_local_steps.push_back(points_local_step3);

        return points_local_steps;
    }

    std::vector<Point_2> get_cone_layout_step(YAML::Node config, std::string step_name, std::vector<Point_2> points_global){
        
        std::vector<Point_2> points_local;

        double car_x = 0.0, car_y = 0.0, car_yaw = 0.0;
        if (config[step_name]) {
            auto car_pose = config[step_name];
            if (car_pose["position"]) {
                car_x = car_pose["position"]["x"].as<double>();
                car_y = car_pose["position"]["y"].as<double>();
            }
            if (car_pose["orientation"]) 
                car_yaw = car_pose["orientation"]["yaw"].as<double>();
        } else 
            std::cerr << "Car pose not found in YAML file." << std::endl;

        // std::cout << "Car's pose at " << step_name << " : " << car_x << " , " << car_y << " , " << car_yaw << std::endl;

        // Convert to car's frame
        for (Point_2 point_global : points_global){
            Point_2 p_local = planner.transform_to_car_frame(point_global,car_x,car_y,car_yaw);
            points_local.push_back(p_local);
        }
        return points_local;
    }

    void print_cone_layout_steps(std::vector<std::vector<Point_2>> cone_layout_steps) {
        // std::cout << "Number of steps: " << cone_layout_steps.size() << std::endl;

        int step = 1;
        for (const auto& cone_layout_step : cone_layout_steps) {
            std::cout << "Step " << step << " : ";
            step++;
            
            for (const auto& point : cone_layout_step) {
                std::cout << "(" << point.x() << ", " << point.y() << ") ";
            }
            std::cout << std::endl;
        }
    }

};

#endif