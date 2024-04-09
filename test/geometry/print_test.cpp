#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "../../include/geometry/d_triang_planner.hpp"


class DTriangPlannerPrintTest {

private:
    DelaunayTriangulation dt;
    DTriangPlanner planner;

public:

    DTriangPlannerPrintTest(){

        setup();

        Edge starting_edge = get_edge(Point_2(15.3254, -5.44316), Point_2(27.2126, -3.01572));
        Edge previous_edge = get_edge(Point_2(15.3254, -5.44316), Point_2(18.8647, 2.8406));
        
        // Call the function to test
        std::vector<Edge> next_edges = planner.get_next_edges(starting_edge, previous_edge);

        // Check the number of next edges
        std::cout << "Number of next edges: " << next_edges.size() << std::endl;


        int true_time = 0; 
        for (auto next_edge : next_edges){
            bool one = are_points_vertices_of_edge(next_edge, Point_2(15.3254, -5.44316), Point_2(19.8133, -10.0369));
            bool two = are_points_vertices_of_edge(next_edge, Point_2(27.2126, -3.01572), Point_2(19.8133, -10.0369));
            if (one || two)
                true_time++;

            // Print out
            print_edge_vertices(next_edge);

        }
    }

    void setup() {
        std::vector<Point_2> points_local;

        // Cones config from near_steep_turn.yaml, converted to car's local frame 
        points_local.push_back(Point_2(8.14437, -4.28801));
        points_local.push_back(Point_2(15.3254, -5.44316));
        points_local.push_back(Point_2(19.8133, -10.0369));
        points_local.push_back(Point_2(17.6274, -16.8421));

        points_local.push_back(Point_2(9.23534, 5.02372));
        points_local.push_back(Point_2(18.8647, 2.8406));
        points_local.push_back(Point_2(27.2126, -3.01572));
        points_local.push_back(Point_2(29.0814, -14.024));

        dt.insert(points_local.begin(), points_local.end()); 
    }

    // Function to find and return a specific edge
    DelaunayTriangulation::Edge get_edge(const Point_2& p1, const Point_2& p2) {
        for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
            auto face = eit->first;
            int i = eit->second;
            Point_2 v1 = face->vertex((i + 1) % 3)->point();
            Point_2 v2 = face->vertex((i + 2) % 3)->point();

            if ((v1 == p1 && v2 == p2) || (v1 == p2 && v2 == p1)) {
                return *eit;
            }
        }
        throw std::runtime_error("Edge not found");
    }

    bool are_points_vertices_of_edge(const DelaunayTriangulation::Edge& edge, const Point_2& p1, const Point_2& p2) {
        auto face = edge.first;
        int i = edge.second;
        Point_2 v1 = face->vertex((i + 1) % 3)->point();
        Point_2 v2 = face->vertex((i + 2) % 3)->point();

        // The == operator checks for exact equality of the coordinates of these points.
        return ((v1 == p1 && v2 == p2) || (v1 == p2 && v2 == p1));
    }

    void print_edge_vertices(const DelaunayTriangulation::Edge& edge) {
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
};

int main() {
    DTriangPlannerPrintTest print_test;
    return 0;
}