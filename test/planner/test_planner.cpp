#include <gtest/gtest.h>
#include "../../src/planner/Planner.h"
#include "../../src/planner/search/DubinsIntegration.h"
#include "dubins.h"

using std::vector;
using std::pair;
using std::cerr;
using std::endl;

TEST(UnitTests, ConstructDubinsIntegrationTest) {
    DubinsIntegration d;

}

TEST(UnitTests, DubinsIntegrationMakePlanTest) {
    DubinsIntegration d;
    State s1(0, 0, 0, 1, 1);
    State s2(0, 5, 0, 1, 6);
    auto v1 = std::make_shared<Vertex>(s1);
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(1, 2);
    EXPECT_DOUBLE_EQ(a, 5);
    auto plan = d.getPlan(e->start()->state(), e->dubinsPath, 1);
    EXPECT_TRUE(plan.getRef().front() == s1);
    EXPECT_GE(plan.getRef().back().y, 4.5); // because of plan density
}

TEST(UnitTests, DubinsIntegrationComputeEdgeCostTest) {
    DubinsIntegration d;
    State s1(0, 0, 0, 1, 1);
    State s2(0, 5, 0, 1, 6);
    auto v1 = std::make_shared<Vertex>(s1);
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    e->computeApproxCost(1, 2);
    Map map;
    DynamicObstacles dynamicObstacles;
    Path path;
    path.add(0, 10);
    double c, t;
    auto newlyCovered = d.computeEdgeCollisionPenaltyAndNewlyCovered(e->dubinsPath, &map, &dynamicObstacles, path, c);
    EXPECT_DOUBLE_EQ(c, 0);
}

TEST(PlannerTests, DubinsWalkTest) {
    // runs forever
    vector<pair<double, double>> points;
    points.emplace_back(10, 10);
    points.emplace_back(20, 10);
    points.emplace_back(20, 20);
    points.emplace_back(10, 20);
    Planner planner(1, 8, Map());
    planner.addToCover(points);
    State start(0,0,M_PI_2,1,1);
    vector<pair<double , double>> newlyCovered;
    while(!points.empty()) {
        if (start.distanceTo(points.front().first, points.front().second) < COVERAGE_THRESHOLD) {
            cerr << "Covered a point near " << start.x << ", " << start.y << endl;
            newlyCovered.push_back(points.front());
            points.erase(points.begin());
        }
        auto plan = planner.plan(newlyCovered, start, DynamicObstacles());
        newlyCovered.clear();
        start = plan[1];
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, PointToPointTest1) {
    vector<pair<double, double>> points;
    points.emplace_back(0, 10);
    points.emplace_back(0, 20);
    points.emplace_back(0, 30);
    Planner planner(2.5, 8, Map());
    planner.addToCover(points);
    State start(0, 0, 0, 2.5, 1);
    auto plan = planner.plan(vector<pair<double, double>>(), start, DynamicObstacles());
    for (auto s : plan) cerr << s.toString() << endl;
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}