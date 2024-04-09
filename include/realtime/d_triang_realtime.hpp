#ifndef DT_REALTIME_H
#define DT_REALTIME_H

#include "d_triang_planner.hpp"


/**
match_new_cones will be called every time step at the rate of the path planner
Inside critical zone, all the cones from last step must be matched
Otherwise, trigger adaptive behavior
    Recompute triangulation 


USAGE: 
    match_new_cones()
        -> true (matched) : reuse_path() 
        -> false : plan new path : replan() 
    get_steering(): Ackerman()

TO RESEARCH:
    Save cone pattern from one last step or multiple last step and compute moving average/use moving median
        Re-plan the path would then be possibly less heavy?
    Use custome Point or Point_2?

COMPENSATE COMPUTER VISION:
    Adaptive Behavior : Replan after a step misses the cones
        May have mechanism for rejecting bad readings inside match_new_cones (check stability of compvision)
            Don't reject to many steps

NOTE: 
runtime_error may throw if the vector are emtpy 
Rembember to process previous data before replanning a new set

**/
class DTRealTime : public DTriangPlanner {

public:

    /*
    Planning workflow
    Returns bool for unit testing only
    */
    bool plan_one_step(const std::vector<Point_2>& cones);

    // void plan_one_step(const std::vector<Point_2>& cones);

    /*
    Get the steering value to publish
    */
    // double get_steering();

    /*
    GPT version
    */
    double get_steering_angle(double x_goal, double y_goal);

    std::vector<Point_2> get_path();


protected:

    bool match_new_cones(const std::vector<Point_2>& cones);

    /*
    If matched
    Find the new corresponding waypoint for each previous path's waypoint
    How: examine the last best path, for each waypoint, get 2 cones, find the corresponding
    2 cones from the new cone pattern, calculate the midpoint, that will be the new corresponding waypoint
    */
    void reuse_path();

    /*
    If not matched, perform triangulation planner from scratch 
    Include: set_cones() -> plan() -> add to _path
    */
    void plan_from_scratch();


protected:

    std::vector<Point_2> _incoming_cones; // Should hand if this is empty
    std::vector<Point_2> _current_cones;
    std::vector<Point_2> _best_path;


protected:

    const double WHEELBASE = 1.580;

    const double CONST_CAR_VEL = 5.0; // (m/s)
    const int PATH_PLANNING_RATE = 30; // (Hz)


    // This should limit by 4 nearest cones ahead
    const double CRITICAL_RANGE = 20.0;   
    const double CRITICAL_RANGE_SQ = pow(CRITICAL_RANGE,2);   

    // Account for both movement of car and error in computervision
    const double error_by_movement_ratio = 1.5;
    const double max_error_by_movement = (CONST_CAR_VEL / PATH_PLANNING_RATE) * error_by_movement_ratio; // (m)
    
    const double MAX_ERROR_BY_COMPUTERVISION = 0.2; // (m)

    const double MATCHING_THRESHOLD = max_error_by_movement + MAX_ERROR_BY_COMPUTERVISION; 
    // = 0.45

    // Goal must not be too near the car
    const double MIN_DIST_GOAL = 5.0;
};

#endif