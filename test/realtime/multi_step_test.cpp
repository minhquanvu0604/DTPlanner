#include <chrono> 
#include "realtime_test_fixture.cpp"


/*
STEP 1 : First step, have to triangulate
STEP 2 : Matched
STEP 3 : Matched
*/
TEST_F(DTRealTimeTestFixture, CoherentSteps) {

    std::vector<std::vector<Point_2>> cone_layout_steps = get_cone_layout_steps("multi_step1.yaml");

    std::vector<Point_2> cone_layout_step1 = cone_layout_steps.at(0);
    std::vector<Point_2> cone_layout_step2 = cone_layout_steps.at(1);
    std::vector<Point_2> cone_layout_step3 = cone_layout_steps.at(2);


    std::cout << "====PRINTING FIRST STEP===="<< std::endl;
    std::vector<std::vector<Point_2>> print_test;
    print_test.push_back(cone_layout_step1);
    print_cone_layout_steps(print_test);
    std::cout << std::endl;

    std::cout << "====START PLANNING===="<< std::endl;
    std::cout << "STEP 1"<< std::endl;
    auto start_time_step1 = std::chrono::high_resolution_clock::now(); 
    bool step1_match = planner.plan_one_step(cone_layout_step1);
    auto end_time_step1 = std::chrono::high_resolution_clock::now(); 

    std::cout << "STEP 2"<< std::endl;
    auto start_time_step2 = std::chrono::high_resolution_clock::now(); 
    bool step2_match = planner.plan_one_step(cone_layout_step2);
    auto end_time_step2 = std::chrono::high_resolution_clock::now(); 
    
    std::cout << "STEP 3"<< std::endl;
    auto start_time_step3 = std::chrono::high_resolution_clock::now(); 
    bool step3_match = planner.plan_one_step(cone_layout_step3);
    auto end_time_step3 = std::chrono::high_resolution_clock::now(); 

    std::cout << "====DONE====" << std::endl;
    print_cone_layout_steps(cone_layout_steps);
    
    auto duration_step1_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_step1 - start_time_step1);
    std::cout << "Execution time for step 1 (nanoseconds): " << duration_step1_ns.count() << " nanoseconds" << std::endl;

    auto duration_step2_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_step2 - start_time_step2);
    std::cout << "Execution time for step 2 (nanoseconds): " << duration_step2_ns.count() << " nanoseconds" << std::endl;

    auto duration_step3_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_step3 - start_time_step3);
    std::cout << "Execution time for step 3 (nanoseconds): " << duration_step3_ns.count() << " nanoseconds" << std::endl;


    ASSERT_FALSE(step1_match); // First step, have to triangulate
    ASSERT_TRUE(step2_match); // Matched
    ASSERT_TRUE(step3_match); // Matched
}


/*
STEP 1 : First step, have to plan
STEP 2 : Missing cone, replan from sratch
STEP 3 : Back to full cones, more info from last step, replan
*/
TEST_F(DTRealTimeTestFixture, SecondStepMissCone) {

    std::vector<std::vector<Point_2>> cone_layout_steps = get_cone_layout_steps("multi_step1.yaml");

    std::vector<Point_2> cone_layout_step1 = cone_layout_steps.at(0);
    std::vector<Point_2> cone_layout_step2 = cone_layout_steps.at(1);
    std::vector<Point_2> cone_layout_step3 = cone_layout_steps.at(2);

    // Miss 1 cone in the second step
    int missing_cone_id = 1;
    cone_layout_step2.erase(cone_layout_step2.begin() + missing_cone_id);


    std::cout << "====PRINTING FIRST STEP===="<< std::endl;
    std::vector<std::vector<Point_2>> print_test;
    print_test.push_back(cone_layout_step1);
    print_cone_layout_steps(print_test);
    std::cout << std::endl;

    std::cout << "====START PLANNING===="<< std::endl;
    std::cout << std::endl << "STEP 1" << std::endl;
    auto start_time_step1 = std::chrono::high_resolution_clock::now(); 
    bool step1_match = planner.plan_one_step(cone_layout_step1);
    auto end_time_step1 = std::chrono::high_resolution_clock::now(); 

    std::cout << std::endl << "STEP 2" << std::endl;
    auto start_time_step2 = std::chrono::high_resolution_clock::now(); 
    bool step2_match = planner.plan_one_step(cone_layout_step2);
    auto end_time_step2 = std::chrono::high_resolution_clock::now(); 
    
    std::cout << std::endl << "STEP 3" << std::endl;
    auto start_time_step3 = std::chrono::high_resolution_clock::now(); 
    bool step3_match = planner.plan_one_step(cone_layout_step3);
    auto end_time_step3 = std::chrono::high_resolution_clock::now(); 

    std::cout << "====DONE====" << std::endl;
    print_cone_layout_steps(cone_layout_steps);
    
    auto duration_step1_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_step1 - start_time_step1);
    std::cout << "Execution time for step 1 (nanoseconds): " << duration_step1_ns.count() << " nanoseconds" << std::endl;

    auto duration_step2_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_step2 - start_time_step2);
    std::cout << "Execution time for step 2 (nanoseconds): " << duration_step2_ns.count() << " nanoseconds" << std::endl;

    auto duration_step3_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time_step3 - start_time_step3);
    std::cout << "Execution time for step 3 (nanoseconds): " << duration_step3_ns.count() << " nanoseconds" << std::endl;


    // ASSERT_FALSE(step1_match); // First step, have to plan
    // ASSERT_FALSE(step2_match); // Missing cone, replan from sratch
    ASSERT_FALSE(step3_match); // Back to full cones, more info from last step, replan
}