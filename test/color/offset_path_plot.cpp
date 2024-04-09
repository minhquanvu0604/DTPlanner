#include "d_triang_planner_color_light_test_fixture.cpp"
#include <opencv2/opencv.hpp>
using namespace cv;

int main()
{
    // std::vector<Points2d> initial_path = {
    //     {100, 100},
    //     {200, 100},
    //     {200, 200},
    //     {100, 200},
    //     {100, 300},
    //     {200, 300},
    //     {400, 400},
    //     {440, 500},
    // };

    std::vector<Point_2> initial_path = {
        {100, 500},
        {150, 400},
        {200, 350},
        {250, 320},
        {300, 300},
        {350, 200},
        {400, 150},
        {440, 100},
    };

    DTriangPlannerColorLight planner;

    // Offset distance and direction
    double offset_distance = 10.0;

    // Calculate the offset path
    std::vector<Point_2> offset_polyline_right = planner.offset_polyline(initial_path, offset_distance, false);
    std::vector<Point_2> offset_polyline_left = planner.offset_polyline(initial_path, offset_distance, true);


    // Create an image to draw the polylines
    int width = 800, height = 800;
    Mat image = Mat::zeros(height, width, CV_8UC3);



    // Draw the original polyline
    for (size_t i = 0; i < initial_path.size() - 1; ++i) {
        cv::line(image, Point(CGAL::to_double(initial_path[i].x()), CGAL::to_double(initial_path[i].y())), Point(CGAL::to_double(initial_path[i+1].x()), CGAL::to_double(initial_path[i+1].y())), Scalar(255, 0, 0), 2);
    }

    for (size_t i = 0; i < offset_polyline_right.size() - 1; ++i) {
        cv::line(image, Point(CGAL::to_double(offset_polyline_right[i].x()), CGAL::to_double(offset_polyline_right[i].y())), Point(CGAL::to_double(offset_polyline_right[i+1].x()), CGAL::to_double(offset_polyline_right[i+1].y())), Scalar(0, 255, 0), 2);
    }

    for (size_t i = 0; i < offset_polyline_left .size() - 1; ++i) {
        cv::line(image, Point(CGAL::to_double(offset_polyline_left[i].x()), CGAL::to_double(offset_polyline_left[i].y())), Point(CGAL::to_double(offset_polyline_left[i+1].x()), CGAL::to_double(offset_polyline_left[i+1].y())), Scalar(0, 0, 255), 2);
    }

    // Show the image
    namedWindow("Polyline", WINDOW_AUTOSIZE);
    imshow("Polyline", image);
    waitKey(0);

    return 0;
}