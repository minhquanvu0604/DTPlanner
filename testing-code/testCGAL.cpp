#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/point_generators_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;

int main() {
    // Example of using the exact kernel
    std::vector<Point_2> points;
    CGAL::Random_points_in_square_2<Point_2> gen(1.0);
    for (int i = 0; i < 10; ++i) {
        points.push_back(*gen++);
    }

    // Perform geometric computations using the exact kernel
    Point_2 p1(1, 2), p2(3, 4);
    Kernel::Segment_2 segment(p1, p2);

    // Use exact predicates
    bool orientation = CGAL::orientation(p1, p2, points[0]) == CGAL::LEFT_TURN;

    // Use exact constructions
    auto result = CGAL::intersection(segment, Kernel::Line_2(points[1], points[2]));

    if (result) {
        if (const Point_2* intersection_point = boost::get<Point_2>(&*result)) {
            std::cout << "Intersection point: " << *intersection_point << std::endl;
        } else {
            std::cout << "No intersection point found." << std::endl;
        }
    } else {
        std::cout << "No intersection found." << std::endl;
    }

    return 0;
}
