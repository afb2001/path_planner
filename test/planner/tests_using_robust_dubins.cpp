#include <gtest/gtest.h>
#include <robust_dubins/RobustDubins.h>
#include "../../src/planner/utilities/Plan.h"
#include "../../src/planner/utilities/StateGenerator.h"
#include "../../src/planner/search/Vertex.h"
#include "../../src/planner/utilities/Path.h"

extern "C" {
#include "dubins.h"
}

using std::vector;
using std::pair;
using std::cerr;
using std::endl;

TEST(DubinsTests, UseRobustDubinsTest) {
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
    for (unsigned long i = 0; i < x.size(); i++){
        std::cout << x[i] << ", " << y[i] << ", " << h[i] << '\n';
    }
    std::cout << endl;

    // alternately, get the RobustDubins::Path object
    // (this contains all the information about the optimal solution)
    RobustDubins::Path optimalPath = rds.get_optimalPath();
    optimalPath.print(); // prints info about this object
}

TEST(Benchmarks, RobustDubinsBenchmark1) {
    double minX, maxX, minY, maxY, minSpeed = 2.5, maxSpeed = 2.5;
    double magnitude = 2.5 * Plan::timeHorizon();
    minX = -magnitude;
    maxX = magnitude;
    minY = -magnitude;
    maxY = magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7);
    auto root = Vertex::makeRoot(generator.generate(), Path());
    for (int i = 0; i < 100000; i++) {
        auto v = Vertex::connect(root, generator.generate());
        v->parentEdge()->computeApproxCost(maxSpeed, 8);
    }
}

TEST(Benchmarks, RobustDubinsBenchmark2) {
    double minX, maxX, minY, maxY, minSpeed = 2.5, maxSpeed = 2.5;
    double magnitude = 2.5 * Plan::timeHorizon();
    minX = -magnitude;
    maxX = magnitude;
    minY = -magnitude;
    maxY = magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7);
    auto start = generator.generate();
    for (int i = 0; i < 100000; i++) {
        auto end = generator.generate();

        RobustDubins::Problem problemStatement;
        problemStatement.set_minTurningRadius(8);

        // Note: heading is in radians
        problemStatement.set_stateInitial(start.x, start.y, start.yaw()); // (x, y, heading)
        problemStatement.set_stateFinal(end.x, end.y, end.yaw()); // (x, y, heading)

        // run the solver
        RobustDubins::Solver rds;
        rds.set_problemStatement(problemStatement);
//    rds.set_numPts(200); // number of waypoints to return (approx.)
        rds.solve();
    }
}

TEST(Benchmarks, DubinsBenchmark1) {
    double minX, maxX, minY, maxY, minSpeed = 2.5, maxSpeed = 2.5;
    double magnitude = 2.5 * Plan::timeHorizon();
    minX = -magnitude;
    maxX = magnitude;
    minY = -magnitude;
    maxY = magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7);
    auto start = generator.generate();
    for (int i = 0; i < 100000; i++) {
        auto end = generator.generate();
        double q0[] = {start.x, start.y, start.yaw()};
        double q1[] = {end.x, end.y, end.yaw()};
        DubinsPath dubinsPath;
        dubins_shortest_path(&dubinsPath, q0, q1, 8);
    }
}

TEST(DubinsTests, DubinsComparison) {
    double minX, maxX, minY, maxY, minSpeed = 2.5, maxSpeed = 2.5;
    double magnitude = 2.5 * Plan::timeHorizon();
    minX = -magnitude;
    maxX = magnitude;
    minY = -magnitude;
    maxY = magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7);
    auto start = generator.generate();
    for (int i = 0; i < 100000; i++) {
        auto end = generator.generate();

        RobustDubins::Problem problemStatement;
        problemStatement.set_minTurningRadius(8);

        // Note: heading is in radians
        problemStatement.set_stateInitial(start.x, start.y, start.yaw()); // (x, y, heading)
        problemStatement.set_stateFinal(end.x, end.y, end.yaw()); // (x, y, heading)

        // run the solver
        RobustDubins::Solver rds;
        rds.set_problemStatement(problemStatement);
//    rds.set_numPts(200); // number of waypoints to return (approx.)
        rds.solve();

        double q0[] = {start.x, start.y, start.yaw()};
        double q1[] = {end.x, end.y, end.yaw()};
        DubinsPath dubinsPath;
        dubins_shortest_path(&dubinsPath, q0, q1, 8);

        EXPECT_LE(fabs(rds.get_optimalPath().get_cost() - dubins_path_length(&dubinsPath)), 0.0001);
    }
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}