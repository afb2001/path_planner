#include <gtest/gtest.h>
#include "../../src/planner/Planner.h"
#include "../../src/planner/search/Edge.h"
#include "../../src/planner/SamplingBasedPlanner.h"
#include <robust_dubins/RobustDubins.h>
//extern "C" {
//#include "dubins.h"
//}

using std::vector;
using std::pair;
using std::cerr;
using std::endl;

TEST(UnitTests, UseRobustDubinsTest) {
    RobustDubins::Problem problemStatement;
    problemStatement.set_minTurningRadius(1);

    // Note: heading is in radians
    problemStatement.set_stateInitial(0,0,0); // (x, y, heading)
    problemStatement.set_stateFinal(10,10,1); // (x, y, heading)
    problemStatement.print(); //

    // run the solver
    RobustDubins::Solver rds;
    rds.set_problemStatement(problemStatement);
//    rds.set_numPts(200); // number of waypoints to return (approx.)
    rds.solve();

    // print results
    rds.print();

    // get waypoints/states of the solution
    std::vector<double> x,y,h; //
    rds.get_optimalWaypointsSetSpacing(x, y, h, 1);
//    rds.get_optimalWaypoints(x,y,h);

    std::cout << '\n';
    for (int i = 0; i < x.size(); i++){
        std::cout << x[i] << ", " << y[i] << ", " << h[i] << '\n';
    }
    std::cout << endl;

    // alternately, get the RobustDubins::Path object
    // (this contains all the information about the optimal solution)
    RobustDubins::Path optimalPath = rds.get_optimalPath();
    optimalPath.print(); // prints info about this object
}

TEST(UnitTests, DubinsIntegrationMakePlanTest) {
    State s1(0, 0, 0, 1, 1);
    State s2(0, 5, 0, 1, 6);
    auto v1 = std::make_shared<Vertex>(s1);
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(1, 2);
    EXPECT_DOUBLE_EQ(a, 5);
    auto plan = e->getPlan(1);
    EXPECT_TRUE(plan.getRef().front() == s1);
    EXPECT_GE(plan.getRef().back().y, 4.5); // because of plan density
}

TEST(UnitTests, DubinsIntegrationComputeEdgeCostTest) {
    State s1(0, 0, 0, 1, 1);
    State s2(0, 5, 0, 1, 6);
    auto v1 = std::make_shared<Vertex>(s1);
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(1, 2);
    Map map;
    DynamicObstacles dynamicObstacles;
    Path path;
    path.add(0, 10);
    auto c = e->computeTrueCost(&map, &dynamicObstacles, 1, 2);
    EXPECT_DOUBLE_EQ(c, a);
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
//        cerr << start.toString() << endl;
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

TEST(PlannerTests, UCSTest1) {
    vector<pair<double, double>> points;
    points.emplace_back(0, 10);
    points.emplace_back(0, 20);
    points.emplace_back(0, 30);
    SamplingBasedPlanner planner(2.5, 8, Map());
    planner.addToCover(points);
    State start(0, 0, 0, 2.5, 1);
    auto plan = planner.plan(vector<pair<double, double>>(), start, DynamicObstacles());
    for (auto s : plan) cerr << s.toString() << endl;
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}