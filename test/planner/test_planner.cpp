#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-err58-cpp"
#include <gtest/gtest.h>
#include "../../src/planner/Planner.h"
#include "../../src/planner/search/Edge.h"
#include "../../src/planner/SamplingBasedPlanner.h"
#include "../../src/planner/AStarPlanner.h"
#include "../../src/common/map/GeoTiffMap.h"
#include "../../src/common/map/GridWorldMap.h"
#include <thread>

using std::vector;
using std::pair;
using std::cerr;
using std::endl;
using std::make_shared;

auto plannerConfig = PlannerConfig(&std::cerr);

TEST(UnitTests, GaussianDensityTest) {
    double sigma[2][2] = {1, 0.1, 0.1, 1};
    double mean[2] = {0, 0};
    Distribution distribution(mean, sigma, 0, 2);
    EXPECT_NEAR(distribution.density(distribution, 2, 0, 0), 0.16, 1e-4);
    EXPECT_NEAR(distribution.density(distribution, 2, 0.6, 0.6), 0.1153, 1e-4);
}

TEST(UnitTests, GaussianInterpolationTest) {
    double sigma1[2][2] = {1, 0.1, 0.1, 1};
    double mean1[2] = {-1, -1};
    double sigma2[2][2] = {1, 0.1, 0.1, 1};
    double mean2[2] = {1, 1};
    Distribution distribution1(mean1, sigma1, 0, 2);
    Distribution distribution2(mean2, sigma2, 0, 4);
    EXPECT_NEAR(distribution1.density(distribution2, 3, 0, 0), 0.16, 1e-4);
    EXPECT_NEAR(distribution1.density(distribution2, 3, 0.6, 0.6), 0.1153, 1e-4);
}

TEST(UnitTests, GaussianTruncateTest) {
    double sigma[2][2] = {{1, 0}, {0, 1}};
    double mean[2] = {0, 0};
    Distribution distribution(mean, sigma, 0, 2);
    // When sigma is the identity matrix, Mahalanobis distance is Euclidean distance, so we should get
    // a zero (truncated) density value for (1.5, 2) (distance to (0, 0) is 2.5)
    EXPECT_DOUBLE_EQ(0, distribution.density(1.5, 2));
    // and a non-zero (non-truncated) density value for (1, 1) (distance to (0, 0) is ~1.414
    EXPECT_LT(0, distribution.density(1, 1));
}

TEST(UnitTests, DynamicObstacleTest1) {
    // This test shouldn't pass right now
    DynamicObstaclesManager obstaclesManager;
    double sigma[2][2] = {{1, 0}, {0, 1}};
    double mean[2] = {0, 0};
    std::vector<Distribution> distributions;
    distributions.emplace_back(mean, sigma, 0, 2);
    distributions.emplace_back(mean, sigma, 0, 3);
    obstaclesManager.update(1, distributions);
    auto p = obstaclesManager.collisionExists(3, 4.5, 10);
    EXPECT_DOUBLE_EQ(p, 0);
    p = obstaclesManager.collisionExists(2.5, 2.5, 10);
    EXPECT_LT(0, p);
    p = obstaclesManager.collisionExists(0, 3.6, 10);
    EXPECT_DOUBLE_EQ(p, 0);
    distributions.clear();
    distributions.emplace_back(mean, sigma, M_PI / 4, 2);
    distributions.emplace_back(mean, sigma, M_PI / 4, 3);
    obstaclesManager.update(1, distributions);
    p = obstaclesManager.collisionExists(0, 3.6, 10);
    EXPECT_LT(0, p);
    distributions.clear();
    // shift and interpolate headings
    double mean1[2] = {1, 1};
    double sigma1[2][2] = {{1, 0}, {0, 1}};
    distributions.emplace_back(mean1, sigma1, M_PI / 6, 2);
    distributions.emplace_back(mean1, sigma1, M_PI / 3, 4);
    obstaclesManager.update(1, distributions);
    auto p1 = obstaclesManager.collisionExists(1, 4.6, 3);
    EXPECT_NEAR(p1, p, 0.00001);
}

TEST(UnitTests, DISABLED_GeoTiffMapTest1) {
    GeoTiffMap map("/home/abrown/Downloads/depth_map/US5NH02M.tiff", -70.71054174878898, 43.073397415457535);
}

TEST(UnitTests, DISABLED_GeoTiffMapTest2) {
    GeoTiffMap map("/home/abrown/Downloads/depth_map/US5NH02M.tiff", -70.71054174878898, 43.073397415457535);
//    EXPECT_DOUBLE_EQ(map.getDepth(0, 0), 0);
//    EXPECT_NEAR(map.getDepth(365000, 4770000), 14.87, 0.001);
    EXPECT_FALSE(map.getUnblockedDistance(0, 0) == -1); // TODO! -- find out what this actually should be somehow
}

TEST(UnitTests, GridWorldMapTest1) {
    GridWorldMap map("/home/alex/Documents/planner_test_suites/test1.map");
//    map.getUnblockedDistance(0, 0);
    EXPECT_DOUBLE_EQ(-1, map.getUnblockedDistance(450, 445));
    EXPECT_DOUBLE_EQ(10, map.getUnblockedDistance(495, 450));
}

TEST(UnitTests, RibbonsTest1) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 0, 1000, 0);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 1000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 1100);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(100, 100, 0), 1000 + sqrt(2)*100);
    ribbonManager.add(0, 20, 1000, 20);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 2100);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 2980);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 2980);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(100, 120, 0), 2000 + sqrt(2)*100);
}

TEST(UnitTests, RibbonsTest2) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(0, 0, 1000, 0);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 1000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 1100);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(100, 100, 0), 1000 + sqrt(2)*100);
    ribbonManager.add(0, 20, 1000, 20);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 2020);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 2120);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 3000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 3000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(100, 120, 0), 2020 + sqrt(2)*100);
}

TEST(UnitTests, RibbonSplitTest) {
    Ribbon r1(40, 100, -70, -120);
    auto r = r1.split(0, 0);
    EXPECT_TRUE(r.length() < 3);
    auto r2 = r1.split(-10, 0);
    EXPECT_EQ(r2.end(), std::make_pair(-10.0, 0.0));
    EXPECT_EQ(r2.start(), std::make_pair(40.0, 100.0));
    EXPECT_EQ(r2.end(), r1.start());
}

TEST(UnitTests, RibbonsTest3) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(0, 0, 1000, 0);
    ribbonManager.cover(2, 0);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(2, 0, 0), 998);
}

TEST(UnitTests, RibbonsTest4) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(0, 0, 1000, 0);
    ribbonManager.cover(1, 1);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1, 0, 0), 999);
}

TEST(UnitTests, RibbonsTest5) {
    RibbonManager ribbonManager(RibbonManager::TspDubinsNoSplitAllRibbons, 8);
    ribbonManager.add(0, 0, 1000, 0);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 1000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 1100);
    // Being lazy using ">=" in examples from previous test instead of figuring out what the real value should be using
    // dubins paths.
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 2000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 2000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(100, 100, 0), 1000 + sqrt(2)*100);
    ribbonManager.add(0, 20, 1000, 20);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 2020);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 2120);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 3000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 3000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(100, 120, 0), 2020 + sqrt(2)*100);
}

TEST(UnitTests, RibbonsTest6) {
    RibbonManager ribbonManager(RibbonManager::TspDubinsNoSplitKRibbons, 8, 2);
    ribbonManager.add(0, 0, 1000, 0);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 1000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 1100);
    // Being lazy using ">=" in examples from previous test instead of figuring out what the real value should be using
    // dubins paths.
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 2000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 2000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(100, 100, 0), 1000 + sqrt(2)*100);
    ribbonManager.add(0, 20, 1000, 20);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 2020);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 2120);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 3000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 3000);
    EXPECT_GE(ribbonManager.approximateDistanceUntilDone(100, 120, 0), 2020 + sqrt(2)*100);
}

TEST(UnitTests, RibbonTest7) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitKRibbons, 8, 2);
    ribbonManager.add(0, 0, 1000, 0);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 1000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 1100);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 2000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(100, 100, 0), 1000 + sqrt(2)*100);
    ribbonManager.add(0, 20, 1000, 20);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 0, 0), 2020);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(-100, 0, 0), 2120);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(0, 1000, 0), 3000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(1000, 1000, 0), 3000);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(100, 120, 0), 2020 + sqrt(2)*100);
}

TEST(UnitTests, RibbonManagerGetNearestEndpointTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(10, 10, 20, 10);
    auto s = ribbonManager.getNearestEndpointAsState(State(9, 9, 0, 0, 0));
    EXPECT_DOUBLE_EQ(s.x(), 10);
    s = ribbonManager.getNearestEndpointAsState(State(10, 10, 0, 0, 0));
    EXPECT_DOUBLE_EQ(s.x(), 20);
    ribbonManager.add(2.6625366957003918, 60, 7.8363094365852275, 60);
    s = ribbonManager.getNearestEndpointAsState(State(7.8363094365852275, 60, 4.7123889803846897, 2.5, 83.397109423209002));
    EXPECT_DOUBLE_EQ(s.x(), 2.6625366957003918);
}

TEST(UnitTests, RibbonManagerFindNearStatesOnRibbonsTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(10, 10, 20, 10);
    ribbonManager.add(2.6625366957003918, 60, 7.8363094365852275, 60);
    auto s1 = State(10.1, 9.9, M_PI_2, 2.5, 1);
    auto states = ribbonManager.findNearStatesOnRibbons(s1, 8);
    for (const auto& s : states) std::cerr << s.toString() << std::endl;
//    double q0[3] = {10.1, 9.9, M_PI_2};
    auto v1 = Vertex::makeRoot(s1, ribbonManager);
    for (auto& s : states) {
        s.speed() = 2.5;
//        double q1[3] = {s.x, s.y, s.yaw()};
//        DubinsPath dubinsPath;
//        int err = dubins_shortest_path(&dubinsPath, q0, q1, 8);
        auto v2 = Vertex::connect(v1, s, 8, true);
        v2->parentEdge()->computeApproxCost();
        v2->parentEdge()->computeTrueCost(plannerConfig);
        auto p = v2->parentEdge()->getPlan(plannerConfig);
//        cerr << p.toString() << endl;
    }
}

TEST(UnitTests, RibbonManagerCoverBetweenTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(10, 10, 20, 10);
    ribbonManager.setRibbonWidth(2);
    ribbonManager.coverBetween(134.778, 62.1946, 133.708, 61.8953);
}

TEST(Benchmarks, RibbonsTSPBenhcmark) {
    auto overallStart = std::chrono::system_clock::now();
    StateGenerator generator(-5000, -5000, 5000, 5000, 0, 0, 19);
    const int step = 1e4;
    const int maxTimes = 1e6;
    for (int i = step; i <= 10 * step; i+= step) {

        RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
        for (int j = 1; j <= i; j++) {
            auto s1 = generator.generate();
            auto s2 = generator.generate();
            ribbonManager.add(s1.x(), s1.y(), s2.x(), s2.y());
        }
        auto startTime = std::chrono::system_clock::now();
        int times;
        for (times = 1; times <= maxTimes; times++) {
            auto s = generator.generate();
            auto d = ribbonManager.approximateDistanceUntilDone(s.x(), s.y(), s.heading());
        }
        auto endTime = std::chrono::system_clock::now();
        auto seconds = (double)((endTime - startTime).count()) / 1e9;
        cerr << i << " ribbons took " << seconds << " seconds"
        << " (" << seconds / i / times << " per ribbon)" << endl;
    }
    auto end = std::chrono::system_clock::now();
    cerr << "Total time: " << (double)((end - overallStart).count()) / 1e9 << " seconds" << endl;
}

TEST(Benchmarks, RibbonsDubinsTSPBenchmark) {
    auto overallStart = std::chrono::system_clock::now();
    StateGenerator generator(-5000, -5000, 5000, 5000, 0, 0, 19);
    const int step = 1e4;
    const int maxTimes = 1e6;
    for (int i = step; i <= 10 * step; i+= step) {

        RibbonManager ribbonManager(RibbonManager::TspDubinsNoSplitAllRibbons, 8);
        for (int j = 1; j <= i; j++) {
            auto s1 = generator.generate();
            auto s2 = generator.generate();
            ribbonManager.add(s1.x(), s1.y(), s2.x(), s2.y());
        }
        auto startTime = std::chrono::system_clock::now();
        int times;
        for (times = 1; times <= maxTimes; times++) {
            auto s = generator.generate();
            auto d = ribbonManager.approximateDistanceUntilDone(s.x(), s.y(), s.heading());
        }
        auto endTime = std::chrono::system_clock::now();
        auto seconds = (double)((endTime - startTime).count()) / 1e9;
        cerr << i << " ribbons took " << seconds << " seconds"
             << " (" << seconds / i / times << " per ribbon)" << endl;
    }
    auto end = std::chrono::system_clock::now();
    cerr << "Total time: " << (double)((end - overallStart).count()) / 1e9 << " seconds" << endl;
}

TEST(Benchmarks, RibbonCoverBenchmark) {
    auto overallStart = std::chrono::system_clock::now();
    StateGenerator generator(-5000, -5000, 5000, 5000, 0, 0, 19);
    const int step = 1e5;
    const int maxTimes = 1e6;
    for (int i = step; i <= 10 * step; i+= step) {

        RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
        for (int j = 1; j <= i; j++) {
            auto s1 = generator.generate();
            auto s2 = generator.generate();
            ribbonManager.add(s1.x(), s1.y(), s2.x(), s2.y());
        }
        auto startTime = std::chrono::system_clock::now();
        int times;
        for (times = 1; times <= maxTimes; times++) {
            auto s = generator.generate();
            ribbonManager.cover(s.x(), s.y());
        }
        auto endTime = std::chrono::system_clock::now();
        auto seconds = (double)((endTime - startTime).count()) / 1e9;
        cerr << i << " ribbons " << maxTimes << " times took " << seconds << " seconds"
             << " (" << seconds / i / times << " per ribbon)" << endl;
    }
    auto end = std::chrono::system_clock::now();
    cerr << "Total time: " << (double)((end - overallStart).count()) / 1e9 << " seconds" << endl;
}

TEST(Benchmarks, RibbonCoverAlongItselfBenchmark) {
    auto overallStart = std::chrono::system_clock::now();
    StateGenerator generator(-5000, -5000, 5000, 5000, 0, 0, 19);
    const int step = 1e0;
    const int maxTimes = 1e4;
    for (int i = step; i <= 10 * step; i+= step) {

        auto startTime = std::chrono::system_clock::now();
        int times;
        for (times = 1; times <= maxTimes; times++) {
            RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
            for (int j = 1; j <= i; j++) {
                auto s1 = generator.generate();
                auto s2 = generator.generate();
                ribbonManager.add(s1.x(), s1.y(), s2.x(), s2.y());
                s1.setHeadingTowards(s2);
                while (s1.distanceTo(s2) > 0.1) {
                    s1 = s1.push(0.1 / 2.5);
                }
            }
        }
        auto endTime = std::chrono::system_clock::now();
        auto seconds = (double)((endTime - startTime).count()) / 1e9;
        cerr << i << " ribbons " << maxTimes << " times took " << seconds << " seconds"
             << " (" << seconds / i / times << " per ribbon)" << endl;
    }
    auto end = std::chrono::system_clock::now();
    cerr << "Total time: " << (double)((end - overallStart).count()) / 1e9 << " seconds" << endl;
}

TEST(UnitTests, MakePlanTest) {
    State s1(0, 0, 0, 1, 1);
    State s2(0, 5, 0, 1, 6);
    auto v1 = Vertex::makeRoot(s1, Path());
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(1, 2);
    EXPECT_DOUBLE_EQ(a, 5);
    // will return dubins wrapper now
    auto plan = e->getPlan(plannerConfig);
    EXPECT_GE(plan.getEndTime() - plan.getStartTime(), 5);
    auto samples = plan.getSamples(0.5);
    for (const auto& s : samples) cerr << s.toString() << endl;
    // TODO
//    EXPECT_TRUE(plan.getRef().front() == s1);
//    EXPECT_GE(plan.getRef().back().y(), 4.5); // because of plan density
}

TEST(UnitTests, ComputeEdgeCostTest) {
    State s1(0, 0, 0, 1, 1);
    State s2(0, 5, 0, 1, 0);
    auto v1 = Vertex::makeRoot(s1, Path());
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(plannerConfig.maxSpeed(), plannerConfig.turningRadius());
    Map::SharedPtr map = make_shared<Map>();
    DynamicObstaclesManager dynamicObstacles;
    Path path;
    path.add(0, 10);
    auto c = e->computeTrueCost(plannerConfig);
    EXPECT_DOUBLE_EQ(3, e->end()->state().time());
    EXPECT_DOUBLE_EQ(c, a);
}

TEST(UnitTests, RunStateGenerationTest) {
    double minX, maxX, minY, maxY, minSpeed = 2.5, maxSpeed = 2.5;
    double magnitude = 2.5 * Plan::timeHorizon();
    minX = -magnitude;
    maxX = magnitude;
    minY = -magnitude;
    maxY = magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7); // lucky seed
    cerr << generator.generate().toString() << endl;
    sleep(1);
}

TEST(UnitTests, VertexTests1) {
    Path path;
    path.add(50, 50);
    auto root = Vertex::makeRoot(State(5, 5, M_PI, 2.5, 1), path);
    auto v1 = Vertex::connect(root, State(5, -20, M_PI, 2.5, 0));
    auto c = v1->parentEdge()->computeApproxCost(2.5, 8);
    EXPECT_DOUBLE_EQ(c, 10);
    Map::SharedPtr m = make_shared<Map>();
    DynamicObstaclesManager obstacles;
    auto t = v1->parentEdge()->computeTrueCost(m, obstacles, 2.5, 8);
    EXPECT_DOUBLE_EQ(t, c);
    EXPECT_DOUBLE_EQ(t, v1->currentCost());
    EXPECT_DOUBLE_EQ(v1->currentCost(), v1->state().time() - 1);
    auto h = v1->computeApproxToGo();
    EXPECT_DOUBLE_EQ(path.maxDistanceFrom(v1->state()) / 2.5, h);
    EXPECT_DOUBLE_EQ(v1->f(), t + h);
}
TEST(UnitTests, VertexTests2) {
    // basically just testing that the ribbons heuristic works
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(30, 30, 50, 50);
    ribbonManager.add(50, 60, 100, 60);
    auto root = Vertex::makeRoot(State(5, 5, M_PI, 2.5, 1), ribbonManager);
    auto v1 = Vertex::connect(root, State(5, -20, M_PI, 2.5, 0));
    auto c = v1->parentEdge()->computeApproxCost(2.5, 8);
    auto m = make_shared<Map>();
    DynamicObstaclesManager obstacles;
    auto t = v1->parentEdge()->computeTrueCost(m, obstacles, 2.5, 8);
    EXPECT_DOUBLE_EQ(c, t);
    auto h = v1->computeApproxToGo();
    EXPECT_DOUBLE_EQ((Path::distance(5, -20, 30, 30) + 20*sqrt(2) + 10 + 50) / 2.5, h);
}

TEST(UnitTests, VertexTests3) {
    // testing the other (max d) ribbon heuristic
    RibbonManager ribbonManager(RibbonManager::MaxDistance);
    ribbonManager.add(30, 30, 50, 50);
    ribbonManager.add(50, 60, 100, 60);
    auto root = Vertex::makeRoot(State(5, 5, M_PI, 2.5, 1), ribbonManager);
    auto v1 = Vertex::connect(root, State(5, -20, M_PI, 2.5, 0));
    v1->parentEdge()->computeApproxCost(2.5, 8);
    auto m = make_shared<Map>();
    DynamicObstaclesManager obstacles;
    v1->parentEdge()->computeTrueCost(m, obstacles, 2.5, 8);
    auto h = v1->computeApproxToGo();
    EXPECT_DOUBLE_EQ((Path::distance(5, -20, 30, 30) + 20*sqrt(2) + 50) / 2.5, h);
}

//TEST(PlannerTests, DISABLED_DubinsWalkTest) {
//    // runs forever // doesn't run forever anymore but fails with new EXPECTs // and fails with the even newer one too
//    // basically all the dubins code is bad
//    vector<pair<double, double>> points;
//    points.emplace_back(100, 100);
//    points.emplace_back(200, 100);
//    points.emplace_back(200, 200);
//    points.emplace_back(100, 200);
//    Planner planner(2.5, 8, make_shared<Map>());
//    planner.addToCover(points);
//    State start(0,0,M_PI_2,2.5,1);
//    vector<pair<double , double>> newlyCovered;
//    vector<State> pastStarts;
//    double lastPlanEndTime = -1;
//    while(!points.empty()) {
////        for (const auto& s : pastStarts) {
////            auto d = s.distanceTo(start);
////            EXPECT_LE(0.5, d);
////            if (d < 0.5) {
////                cerr << "Circled back to " << s.time << " from " << start.time << endl;
////            }
////
////        }
////        EXPECT_LE(start.x, 216);
////        EXPECT_LE(start.y, 216);
//        pastStarts.push_back(start);
//        cerr << start.toString() << endl;
//        if (Path::covers(points.front(), start.x, start.y)) {
//            cerr << "Covered a point near " << start.x << ", " << start.y << endl;
//            newlyCovered.push_back(points.front());
//            points.erase(points.begin());
//        }
//        auto plan = planner.plan(newlyCovered, start, DynamicObstaclesManager(), 0);
//        if (lastPlanEndTime != -1) {
//            EXPECT_LE(plan.back().time, lastPlanEndTime + 0.5); // 0.5 added for small fluctuations
//        }
//        lastPlanEndTime = plan.back().time;
//        newlyCovered.clear();
//        start = plan[1];
//    }
//}

//TEST(PlannerTests, PointToPointTest1) {
//    vector<pair<double, double>> points;
//    points.emplace_back(0, 10);
//    points.emplace_back(0, 20);
//    points.emplace_back(0, 30);
//    Planner planner(2.5, 8, make_shared<Map>());
//    planner.addToCover(points);
//    State start(0, 0, 0, 2.5, 1);
//    auto plan = planner.plan(vector<pair<double, double>>(), start, DynamicObstaclesManager(), 0);
//    for (auto s : plan) cerr << s.toString() << endl;
//    EXPECT_DOUBLE_EQ(plan.back().time, 12.64);
//}

//TEST(PlannerTests, UCSTest1) {
//    vector<pair<double, double>> points;
//    points.emplace_back(0, 10);
//    points.emplace_back(0, 20);
//    points.emplace_back(0, 30);
//    SamplingBasedPlanner planner(2.5, 8, make_shared<Map>());
//    planner.addToCover(points);
//    State start(0, 0, 0, 2.5, 1);
//    auto plan = planner.plan(vector<pair<double, double>>(), start, DynamicObstaclesManager(), 0);
//    for (auto s : plan) cerr << s.toString() << endl;
//}

// TODO! -- port to ribbons
TEST(PlannerTests, VertexQueueTests) {
//    State start(0, 0, 0, 2.5, 1);
//    Path path;
//    path.add(0, 10);
//    path.add(0, 20);
//    path.add(0, 30);
//    Map::SharedPtr m = make_shared<Map>();
//    DynamicObstaclesManager obstacles;
//    auto root = Vertex::makeRoot(start, path);
//    AStarPlanner planner(2.5, 8, m);
//    planner.addToCover(path.get());
//    planner.pushVertexQueue(root);
//    auto popped = planner.popVertexQueue();
//    EXPECT_EQ(root, popped);
//    State s1(0, 10, 0, 2.5, 0), s2(0, -10, M_PI, 2.5, 0);
//    auto v1 = Vertex::connect(root, s1);
//    v1->parentEdge()->computeTrueCost(m, obstacles, 2.5, 8);
//    v1->setCurrentCost();
//    auto v2 = Vertex::connect(root, s2);
//    v2->parentEdge()->computeTrueCost(m, obstacles, 2.5, 8);
//    v2->setCurrentCost();
//    planner.pushVertexQueue(v1);
//    planner.pushVertexQueue(v2);
//    popped = planner.popVertexQueue();
//    EXPECT_EQ(v1, popped);
}

TEST(UnitTests, EmptyVertexQueueTest) {
    AStarPlanner planner;
    EXPECT_THROW(planner.popVertexQueue(), std::out_of_range);
}

//TEST(UnitTests, ExpandTest1) {
//    State start(0, 0, 0, 2.5, 1);
//    Path path;
//    path.add(0, 10);
//    path.add(0, 20);
//    path.add(0, 30);
//    Map::SharedPtr m = make_shared<Map>();
//    DynamicObstaclesManager obstacles;
//    auto root = Vertex::makeRoot(start, path);
//    AStarPlanner planner(2.5, 8, 2.5, 16, m);
//    planner.addToCover(path.get());
//    StateGenerator generator(-50, 50, -50, 50, 2.5, 2.5, 7);
//    planner.addSamples(generator, 1000);
//    planner.expand(root, &obstacles);
//    double fPrev = 0;
//    for (int i = 0; i < 20; i++) { // k + 1, at the time of this writing
//        auto v = planner.popVertexQueue();
////        cerr << "Popped vertex at " << v->state().toString() << endl;
////        cerr << "g = " << v->currentCost() << ", h = " << v->approxToGo() << ", f = " << v->f() << endl;
//        EXPECT_LE(fPrev, v->f());
//        fPrev = v->f();
//    }
//    EXPECT_THROW(planner.popVertexQueue(), std::out_of_range);
//}

TEST(UnitTests, ExpandTest1Ribbons) {
    StateGenerator generator(-50, 50, -50, 50, 2.5, 2.5, 9);
    State start = generator.generate();
    start.time() = 1;
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    DynamicObstaclesManager obstacles;
    auto root = Vertex::makeRoot(start, ribbonManager);
    AStarPlanner planner;
    planner.setConfig(plannerConfig);
    planner.addSamples(generator, 1000);
    planner.expand(root, obstacles);
    double fPrev = 0;
    for (int i = 0; i < 20; i++) { // k + 1, at the time of this writing
        auto v = planner.popVertexQueue();
        cerr << v->toString() << endl;
        EXPECT_LE(fPrev, v->f());
        fPrev = v->f();
    }
    EXPECT_THROW(planner.popVertexQueue(), std::out_of_range);
}

TEST(UnitTests, ExpandDifferentTurningRadiiTest) {
    State start(0, 0, 0, 2.5, 1);
    RibbonManager ribbonManager;
    ribbonManager.add(0, 0, 0, 30);
    Map::SharedPtr m = make_shared<Map>();
    DynamicObstaclesManager obstacles;
    auto root = Vertex::makeRoot(start, ribbonManager);
    AStarPlanner planner;
    planner.setConfig(plannerConfig);
    StateGenerator generator(-50, 50, -50, 50, 2.5, 2.5, 7);
    planner.addSamples(generator, 1000);
    planner.expand(root, obstacles);
    auto v1 = planner.popVertexQueue();
    EXPECT_TRUE(v1->coverageAllowed());
}

TEST(UnitTests, AngleConsistencyTest) {
    auto minAngleChange = Edge::dubinsIncrement() / plannerConfig.turningRadius() + 1e-5; // arc length / radius + tolerance
    StateGenerator generator(-50, 50, -50, 50, 2.5, 2.5, 7);
    auto rootState = generator.generate();
    rootState.time() = 1;
    rootState.speed() = plannerConfig.maxSpeed();
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 20, 10);
    auto root = Vertex::makeRoot(rootState, ribbonManager);
    for (int i = 0; i < 10; i++) {
        auto startState = generator.generate();
        auto start = Vertex::connect(root, startState);
        start->parentEdge()->computeTrueCost(plannerConfig);
        auto endState = generator.generate();
        auto end = Vertex::connect(start, endState);
        end->parentEdge()->computeTrueCost(plannerConfig);
        auto sPrev = startState;
        for (const auto& s : end->parentEdge()->getPlan(plannerConfig).getSamples(Edge::dubinsIncrement() / plannerConfig.maxSpeed())) {
            ASSERT_NEAR(sPrev.headingDifference(s.heading()), 0, minAngleChange);
            sPrev = s;
        }
        EXPECT_NEAR(sPrev.heading(), end->state().heading(), minAngleChange);

    }
}

TEST(PlannerTests, RHRSAStarTest1Ribbons) {
    plannerConfig.now();
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.95);
    EXPECT_FALSE(plan.empty());
//    for (auto s : plan.getHalfSecondSamples()) cerr << s.toString() << endl;
}

TEST(PlannerTests, RHRSAStarTest2Ribbons) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    while(!ribbonManager.done()) {
        ribbonManager.cover(start.x(), start.y());
        auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.5); // quick iterations
        ASSERT_FALSE(plan.empty());
        start = plan.getHalfSecondSamples()[1];
        ASSERT_LT(start.time(), 30);
        cerr << start.toString() << endl;
    }
}

//TEST(PlannerTests, RHRSAStarTest3) {
//    // no memory leak with Valgrind
//    Path path;
//    path.add(10, 10);
//    path.add(20, 10);
//    path.add(20, 20);
//    path.add(10, 20);
//    AStarPlanner planner(2.5, 8, 2.5, 16, make_shared<Map>());
//    planner.addToCover(path.get());
//    State start(0, 0, 0, 1, 1);
//    while(path.size() != 0) {
//        cerr << start.toString() << endl;
//        auto newlyCovered = path.removeNewlyCovered(start.x, start.y);
//        if (!newlyCovered.empty()) {
//            cerr << "Covered a point near " << start.x << ", " << start.y << endl;
//        }
//        auto plan = planner.plan(newlyCovered, start, DynamicObstaclesManager(), 0.95);
//        ASSERT_FALSE(plan.empty());
//        start = plan[1];
//        ASSERT_LT(start.time, 60);
//    }
//}

TEST(PlannerTests, RHRSAStarTest4Ribbons) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    bool headingChanged = false;
    while(!ribbonManager.done()) {
        if (!headingChanged) ribbonManager.cover(start.x(), start.y());
        auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.95);
        ASSERT_FALSE(plan.empty());
        headingChanged = plan.getHalfSecondSamples()[1].heading() == start.heading();
        start = plan.getHalfSecondSamples()[1];
        ASSERT_LT(start.time(), 180);
        cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, RHRSAStarTest4aRibbons) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_test_visualizations"));
    plannerConfig.setVisualizations(true);
    plannerConfig.setVisualizer(&visualizer);
    auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.95);
    EXPECT_FALSE(plan.empty());
    for (auto s : plan.getHalfSecondSamples()) cerr << s.toString() << endl;
}

TEST(PlannerTests, RHRSAStarSingleRibbonTSP) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(0, 20, 50, 20);
    ribbonManager.add(0, 40, 50, 40);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    bool headingChanged = true; // assume coverage
    while(!ribbonManager.done()) {
        /*if (!headingChanged)*/ ribbonManager.cover(start.x(), start.y());
        auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.95);
        ASSERT_FALSE(plan.empty());
//        headingChanged = plan.getHalfSecondSamples()[1].heading() == start.heading();
        start.time() += 1;
        plan.sample(start);
        for (const auto& s : plan.getHalfSecondSamples()) cerr << s.toString() << endl;
        ASSERT_LT(start.time(), 180);
        cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
        cerr << start.toString() << endl;
    }

}

TEST(PlannerTests, RHRSAStarTest5TspRibbons) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitKRibbons, 8, 2);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    bool headingChanged = false;
    while(!ribbonManager.done()) {
        if (!headingChanged) ribbonManager.cover(start.x(), start.y());
        auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.95);
        ASSERT_FALSE(plan.empty());
        headingChanged = plan.getHalfSecondSamples()[1].heading() == start.heading();
        start = plan.getHalfSecondSamples()[1];
        ASSERT_LT(start.time(), 180);
        cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, RHRSAStarTest6DubinsRibbons) {
    RibbonManager ribbonManager(RibbonManager::TspDubinsNoSplitKRibbons, 8, 2);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    bool headingChanged = false;
    while(!ribbonManager.done()) {
        if (!headingChanged) ribbonManager.cover(start.x(), start.y());
        auto plan = planner.plan(ribbonManager, start, plannerConfig, 0.95);
        ASSERT_FALSE(plan.empty());
        headingChanged = plan.getHalfSecondSamples()[1].heading() == start.heading();
        start = plan.getHalfSecondSamples()[1];
        ASSERT_LT(start.time(), 180);
        cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, RHRSAStarSeparateThreadTest) {
    auto planner = std::unique_ptr<Planner>(new AStarPlanner);
    RibbonManager ribbonManager;
    ribbonManager.add(10, 10, 20, 10);
    ribbonManager.add(20, 10, 20, 20);
    ribbonManager.add(20, 30, 10, 30);
    ribbonManager.add(10, 20, 10, 10);
    State start(0, 0, 0, 1, 1);
    std::thread t ([&]{
        while(!ribbonManager.done()) {
            ribbonManager.cover(start.x(), start.y());
            auto plan = planner->plan(ribbonManager, start, plannerConfig, 0.95);
            ASSERT_FALSE(plan.empty());
            start = plan.getHalfSecondSamples()[1];
            ASSERT_LT(start.time(), 60);
            cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
            cerr << start.toString() << endl;
        }
    });
    t.join();
}

TEST(PlannerTests, VisualizationTest) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitKRibbons, 8, 2);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    PlannerConfig config(&cerr);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_visualizations"));
    config.setVisualizer(&visualizer);
    config.setVisualizations(true);
    config.setNowFunction([] () -> double {
        struct timespec t{};
        clock_gettime(CLOCK_REALTIME, &t);
        return t.tv_sec + t.tv_nsec * 1e-9;
    });
    config.setMap(make_shared<Map>());
    config.setObstacles(DynamicObstaclesManager());
    config.setBranchingFactor(4);
    auto plan = planner.plan(ribbonManager, start, config, 0.95);
    EXPECT_FALSE(plan.empty());
}

TEST(PlannerTests, VisualizationLongerTest) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons, 8, 2);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    PlannerConfig config(&cerr);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_visualizations"));
    config.setVisualizer(&visualizer);
    config.setVisualizations(true);
    config.setNowFunction([] () -> double {
        struct timespec t{};
        clock_gettime(CLOCK_REALTIME, &t);
        return t.tv_sec + t.tv_nsec * 1e-9;
    });
    config.setMap(make_shared<Map>());
    config.setObstacles(DynamicObstaclesManager());
    std::thread t ([&]{
        while(!ribbonManager.done()) {
            ribbonManager.cover(start.x(), start.y());
            auto plan = planner.plan(ribbonManager, start, config, 0.95);
            ASSERT_FALSE(plan.empty());
            start = plan.getHalfSecondSamples()[1];
            ASSERT_LT(start.time(), 60);
            cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
            cerr << start.toString() << endl;
        }
    });
    t.join();
}

TEST(PlannerTests, RandomVisualizationTest) {
    RibbonManager ribbonManager(RibbonManager::MaxDistance, 8, 2);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    PlannerConfig config(&cerr);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_visualizations"));
    config.setVisualizer(&visualizer);
    config.setVisualizations(true);
    config.setNowFunction([] () -> double {
        struct timespec t{};
        clock_gettime(CLOCK_REALTIME, &t);
        return t.tv_sec + t.tv_nsec * 1e-9;
    });
    config.setMap(make_shared<Map>());
    config.setObstacles(DynamicObstaclesManager());
    StateGenerator generator(-10, 30, 0, 120, 2.5, 2.5, 9);
    for (int i = 0; i < 10; i++) {
        auto plan = planner.plan(ribbonManager, generator.generate(), config, 0.95);
        ASSERT_FALSE(plan.empty());
    }
}

TEST(PlannerTests, RandomPointsTest) {
    // written for profiling
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitKRibbons, 8, 2);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    StateGenerator generator(-10, 30, 0, 120, 2.5, 2.5, 9);
    for (int i = 0; i < 10; i++) {
        auto plan = planner.plan(ribbonManager, generator.generate(), plannerConfig, 9.95);
        ASSERT_FALSE(plan.empty());
    }
}


int main(int argc, char **argv){
    auto f = [] () -> double {
        struct timespec t{};
        clock_gettime(CLOCK_REALTIME, &t);
        return t.tv_sec + t.tv_nsec * 1e-9;
    };
    plannerConfig.setNowFunction(f);
    plannerConfig.setMap(make_shared<Map>());
    plannerConfig.setObstacles(DynamicObstaclesManager());
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#pragma clang diagnostic pop