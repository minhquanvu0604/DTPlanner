#include <gtest/gtest.h>
#include "d_triang_planner_color_light.hpp"

// Define a test fixture
class DTriangPlannerColorLightTest : public ::testing::Test {

protected:
    std::shared_ptr<DTriangPlannerColorLight> planner;

protected:
    void SetUp() override {
        planner = std::make_shared<DTriangPlannerColorLight>();
    }

    void TearDown() override {
        // Clean up after the tests
    }
};
