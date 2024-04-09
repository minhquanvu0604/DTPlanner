#include "geometry_test_fixture.cpp"

TEST_F(DTriangPlannerTestFixture, ReturnFirstEdgeDefaultCones) {

    Edge first_edge = _planner.triangulate();
    //print_edge_vertices(first_edge);

    Point_2 first_edge_p1(9.23534, 5.02372);
    Point_2 first_edge_p2(8.14437, -4.28801);

    bool correct = are_points_vertices_of_edge(first_edge, first_edge_p1, first_edge_p2);

    ASSERT_TRUE(correct);
}

TEST_F(DTriangPlannerTestFixture, ReturnFirstEdgeConeLayout1) {

    std::vector<Point_2> points_local = get_cone_layout("straight.yaml");
    _planner.set_cones(points_local);

    Edge first_edge = _planner.triangulate();
    // print_edge_vertices(first_edge);

    Point_2 first_edge_p1(5.88831, 4.29521);
    Point_2 first_edge_p2(5.83438, -3.69787);

    bool correct = are_points_vertices_of_edge(first_edge, first_edge_p1, first_edge_p2);

    ASSERT_TRUE(correct);
}

TEST_F(DTriangPlannerTestFixture, ReturnFirstEdgeConeLayout2) {

    std::vector<Point_2> points_local = get_cone_layout("straight_with_stretch_p.yaml");
    _planner.set_cones(points_local);

    Edge first_edge = _planner.triangulate();
    // print_edge_vertices(first_edge);

    Point_2 first_edge_p1(5.88831, 4.29521);
    Point_2 first_edge_p2(5.83438, -3.69787);

    bool correct = are_points_vertices_of_edge(first_edge, first_edge_p1, first_edge_p2);

    ASSERT_TRUE(correct);
}

TEST_F(DTriangPlannerTestFixture, ReturnFirstEdgeConeLayout3) {

    std::vector<Point_2> points_local = get_cone_layout("straight_missing_cone.yaml");
    _planner.set_cones(points_local);

    Edge first_edge = _planner.triangulate();
    // print_edge_vertices(first_edge);

    Point_2 first_edge_p1(5.88831, 4.29521);
    Point_2 first_edge_p2(5.83438, -3.69787);

    bool correct = are_points_vertices_of_edge(first_edge, first_edge_p1, first_edge_p2);

    ASSERT_TRUE(correct);
}

TEST_F(DTriangPlannerTestFixture, ReturnFirstEdgeConeLayout4) {

    std::vector<Point_2> points_local = get_cone_layout("slight_curve.yaml");
    _planner.set_cones(points_local);

    Edge first_edge = _planner.triangulate();
    // print_edge_vertices(first_edge);

    Point_2 first_edge_p1(6.89977 , -3.72891);
    Point_2 first_edge_p2(6.06527 , 4.29646);

    bool correct = are_points_vertices_of_edge(first_edge, first_edge_p1, first_edge_p2);

    ASSERT_TRUE(correct);
}

TEST_F(DTriangPlannerTestFixture, ReturnFirstEdgeConeLayout5) {

    std::vector<Point_2> points_local = get_cone_layout("near_steep_turn.yaml");
    _planner.set_cones(points_local);

    Edge first_edge = _planner.triangulate();
    // print_edge_vertices(first_edge);

    Point_2 first_edge_p1(9.23534, 5.02372);
    Point_2 first_edge_p2(8.14437, -4.28801);

    bool correct = are_points_vertices_of_edge(first_edge, first_edge_p1, first_edge_p2);

    ASSERT_TRUE(correct);
}


