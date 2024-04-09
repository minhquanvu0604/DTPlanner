#include "d_triang_planner_color_light_test_fixture.cpp"

TEST_F(DTriangPlannerColorLightTest, OffsetPathTest1) {

    std::vector<Point_2> initialPath = {
        {100, 500},
        {150, 400},
        {200, 350},
        {250, 320},
        {300, 300},
        {350, 200},
        {400, 150},
        {440, 100},
    };

     // Offset distance and direction
    double offsetDistance = 10.0;

    // Calculate the offset path
    std::vector<Point_2> offset_polyline_right = planner->offset_polyline(initialPath, offsetDistance, false);
    std::vector<Point_2> offset_polyline_left = planner->offset_polyline(initialPath, offsetDistance, true);

    std::cout << "Offset left:" << std::endl;
    planner->print_path(offset_polyline_left);

    std::cout << "Offset right:" << std::endl;
    planner->print_path(offset_polyline_right);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
