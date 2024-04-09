#include "geometry_test_fixture.cpp"


TEST_F(DTriangPlannerTestFixture, ExpandTest1) {

    _planner.plan();

    std::vector<std::vector<Point_2>> paths = _planner.get_paths();

    std::cout << "Print best path 2: " << std::endl;
    auto best_path_2 = _planner.get_best_path_2();
    _planner.print_path_2(best_path_2);

    
    ASSERT_TRUE(paths.size() > 0);
}