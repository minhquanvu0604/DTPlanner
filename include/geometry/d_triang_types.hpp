#ifndef D_TRIANG_TYPES_H
#define D_TRIANG_TYPES_H

#include <vector>
#include <memory>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef DelaunayTriangulation::Edge Edge;
typedef DelaunayTriangulation::Edge_circulator Edge_circulator;
typedef CGAL::Segment_2<Kernel> Segment;
// DelaunayTriangulation::Face_handle Face_handle; // somehow doesn't work


struct Pose{
    Point_2 position;
    double yaw;

    Pose(Point_2 position, double yaw) : position{position}, yaw{yaw} {}
    Pose(double x, double y, double yaw) : position{Point_2(x, y)}, yaw{yaw} {}
};

namespace DT {



    /*
    Store information of each node
        [x] Parent node pointer
        [x] Children nodes pointer
        [ ] Node-wise confidence score
        [x] 2 cones that make up this node
            ( ) Calculate the synergy of all nodes in each path to evaluate confidence score of the path
            (x) For matching algorithm: the matching next pattern will have to use only certain cones 
                that corresponds to the last cones used to find midpoints 
        []


    
    
    A path is a vector of Nodes
    
    For colorblind mode
    */
    struct Node {

        // Mid point
        // Direction from the last Node to this Node
        // Used to validate expansion and storing paths
        // 
        // Ther's an approach where midpoints are calculated all at once,
        //  and each Node can store a pointer to an existing mid points
        // This avoids data duplication but efforts would be made to find that midpoint 
        //  to point to given an edge
        // However, it reduces memory by not storing different variables for the same point,
        //  maybe easy to find the same point (check if the pointer points to the same memory)
        Pose pose;  

        // std::pair<Edge, Edge> traverse_state; // Used to navigate to new node to expand the tree
        std::shared_ptr<Node> parent_ptr;
        std::vector<std::shared_ptr<Node>> children_ptrs;

        // // Similar to Pose's Point_2, can be a pointer to Segment or two pointers to Vertex
        std::array<Point_2, 2> cones;


        // double depth = 0; // DO I NEED THIS?
        // double absTurning = 0; // DO I NEED THIS?
        // double reward = 0; // DO I NEED THIS? REWARD IS FOR FULL PATH
        // double accumulate_angle_diff = -1.0;

        //Edge edge; // Each Node is a mid point associated with an edge

        // std::vector<std::shared_ptr<Node>> children;
        // std::shared_ptr<Node> parent;

        Node(const Pose& p) : pose{p} {}
        Node(double x, double y, double yaw) : pose{Pose(x, y , yaw)} {}
            
        // double angle_difference() const {
        //     if (parent) {
        //         double angle_diff = std::atan2(std::sin(pose.yaw - parent->pose.yaw), std::cos(pose.yaw - parent->pose.yaw));
        //         return std::abs(angle_diff);
        //     }
        //     return -100.0; 
        // }
    };

    struct TraverseState{

        TraverseState() = default;
        TraverseState(std::shared_ptr<Node> node, Edge current_edge, Edge previous_edge)
            : node_ptr{node}, current_edge{current_edge}, previous_edge{previous_edge} {}

        std::shared_ptr<Node> node_ptr;
        Edge current_edge;    
        Edge previous_edge;
        
    };

    /*
    What parameters to represent a path?
    - Associate edge for each point?
    - A map to represent which nodes the path goes between (good path should unlock more cones)?
    - Per-path basis or per-node basis: angle difference, confident point (depends on path eval metrics)?
    - Depth 
    - etc
    */
    // struct Path {
    //     std::vector<std::shared_ptr<Point_2>> order;
    //     int depth = -1;
    // };

    // struct Path{

    // }
}

namespace DTCL {

    struct Cone {
        Cone(Point_2 point, int color) : point(point), color(color) {}
        
        Point_2 point;
        int color; // 0 is blue-left, 1 is yellow-right
    };

    struct Node {
        Pose pose;  

        std::shared_ptr<Node> parent_ptr;
        std::vector<std::shared_ptr<Node>> children_ptrs;

        // // // Similar to Pose's Point_2, can be a pointer to Segment or two pointers to Vertex
        // std::array<Point_2, 2> cones;



        Node(const Pose& p) : pose{p} {}
        Node(double x, double y, double yaw) : pose{Pose(x, y , yaw)} {}
    };
    
    struct TraverseState{

        TraverseState() = default;
        TraverseState(std::shared_ptr<Node> node, Edge current_edge, Edge previous_edge)
            : node_ptr{node}, current_edge{current_edge}, previous_edge{previous_edge} {}

        std::shared_ptr<Node> node_ptr;
        Edge current_edge;    
        Edge previous_edge;
        
    };
}


#endif