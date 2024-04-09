#include "geometry_test_fixture.cpp"

TEST_F(DTriangPlannerTestFixture, GetNextEdges1) {

    _planner.triangulate();

    Point_2 starting_edge_p1(15.3254, -5.44316);
    Point_2 starting_edge_p2(27.2126, -3.01572);
    
    Point_2 previous_edge_p1(18.8647, 2.8406);
    Point_2 previous_edge_p2(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());


    Point_2 next_edge1_p1(15.3254, -5.44316);
    Point_2 next_edge1_p2(19.8133, -10.0369);

    Point_2 next_edge2_p1(27.2126, -3.01572);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, GetNextEdges2) {

    _planner.triangulate();

    Point_2 starting_edge_p1(27.2126, -3.01572);
    Point_2 starting_edge_p2(19.8133, -10.0369);

    Point_2 previous_edge_p1(15.3254, -5.44316);    
    Point_2 previous_edge_p2(27.2126, -3.01572);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(27.2126, -3.01572);
    Point_2 next_edge1_p2(29.0814, -14.024);

    Point_2 next_edge2_p1(29.0814, -14.024);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, GetNextEdges3) {

    _planner.triangulate();

    Point_2 starting_edge_p1(27.2126, -3.01572);
    Point_2 starting_edge_p2(19.8133, -10.0369);

    Point_2 previous_edge_p1(29.0814, -14.024);    
    Point_2 previous_edge_p2(19.8133, -10.0369);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(27.2126, -3.01572);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(15.3254, -5.44316);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, GetNextEdges4) {

    _planner.triangulate();

    Point_2 starting_edge_p1(15.3254, -5.44316);
    Point_2 starting_edge_p2(17.6274, -16.8421);

    Point_2 previous_edge_p1(19.8133, -10.0369);    
    Point_2 previous_edge_p2(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(8.14437, -4.28801);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(17.6274, -16.8421);
    Point_2 next_edge2_p2(8.14437, -4.28801);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, GetNextEdges5) {

    _planner.triangulate();

    Point_2 starting_edge_p1(15.3254, -5.44316);
    Point_2 starting_edge_p2(19.8133, -10.0369);

    Point_2 previous_edge_p1(17.6274, -16.8421);
    Point_2 previous_edge_p2(15.3254, -5.44316);    

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(27.2126, -3.01572);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(27.2126, -3.01572);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, FirstEdge1) {

    _planner.triangulate();

    Point_2 starting_edge_p1(9.23534, 5.02372);
    Point_2 starting_edge_p2(8.14437, -4.28801);  
    Point_2 another_p_inside(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, another_p_inside);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge;
    

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(9.23534, 5.02372);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(8.14437, -4.28801);
    Point_2 next_edge2_p2(15.3254, -5.44316);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, FirstEdge2) {

    _planner.triangulate();

    Point_2 starting_edge_p1(29.0814, -14.024);
    Point_2 starting_edge_p2(17.6274, -16.8421);  
    Point_2 another_p_inside(19.8133, -10.0369);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, another_p_inside);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge;


    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(29.0814, -14.024);
    Point_2 next_edge1_p2(19.8133, -10.0369);

    Point_2 next_edge2_p1(17.6274, -16.8421);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, FirstEdge3) {

    _planner.triangulate();

    Point_2 starting_edge_p1(18.8647, 2.8406);
    Point_2 starting_edge_p2(27.2126, -3.01572);  
    Point_2 another_p_inside(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, another_p_inside);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge;
    

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(18.8647, 2.8406);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(27.2126, -3.01572);
    Point_2 next_edge2_p2(15.3254, -5.44316);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTestFixture, InfiniteEdgeCheck1) {

    _planner.triangulate();

    Point_2 starting_edge_p1(18.8647 , 2.8406);
    Point_2 starting_edge_p2(27.2126 , -3.01572);

    Point_2 previous_edge_p1(15.3254 , -5.44316);    
    Point_2 previous_edge_p2(18.8647 , 2.8406);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = _planner.get_next_edges(current_edge, previous_edge);

    // for (auto edge : next_edges){
    //     print_edge_vertices(edge);
    // }

    bool one = _dt->is_infinite(next_edges.at(0));
    bool two = _dt->is_infinite(next_edges.at(1));
    bool three = _dt->is_infinite(next_edges.at(2));

    // int true_times = 0;
    // if (one){
    //     true_times++;
    //     std::cout << "1" << std::endl;
    // }
    // if (two){
    //     true_times++;
    //     std::cout << "2" << std::endl;
    // }
    // if (three){
    //     true_times++;
    //     std::cout << "3" << std::endl;
    // }     
    // ASSERT_EQ(true_times,2);

    ASSERT_FALSE(one);
    ASSERT_TRUE(two);
    ASSERT_TRUE(three);
}
