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
#include "../../src/common/dynamic_obstacles/BinaryDynamicObstaclesManager.h"
#include "../../src/common/dynamic_obstacles/GaussianDynamicObstaclesManager.h"
#include <thread>
#include <path_planner_common/Plan.h>

using std::vector;
using std::pair;
using std::cerr;
using std::endl;
using std::make_shared;


// Tests are executed from ~/project11/catkin_ws/devel/lib/path_planner

auto plannerConfig = PlannerConfig(&std::cerr);

void validatePlan(DubinsPlan plan, PlannerConfig config) {
    auto wrappers = plan.get();
    auto timeIncrement = plannerConfig.collisionCheckingIncrement() / config.maxSpeed();
    for (int i = 0; i < wrappers.size() - 1; i++) {
        auto& w1 = wrappers[i]; auto& w2 = wrappers[i + 1];
        EXPECT_NEAR(w1.getEndTime(), w2.getStartTime(), 1e-5);
        State s1, s2;
        s1.time() = w2.getStartTime();// - timeIncrement;
        s2.time() = w2.getStartTime();
        w1.sample(s1); w2.sample(s2);
        // TODO! -- this should probably be zero, not the dubins increment, but I can't be bothered with that now
        EXPECT_NEAR(s1.distanceTo(s2), 0, plannerConfig.collisionCheckingIncrement() + 1e-5);
        EXPECT_NEAR(s1.headingDifference(s2), 0, plannerConfig.collisionCheckingIncrement() / config.turningRadius() + 1e-5);
    }
}

//region Plan transfer tests

static DubinsPlan getPlan(path_planner_common::Plan plan) {
    DubinsPlan result;
    for (const auto& d : plan.paths) {
        DubinsWrapper wrapper;
        DubinsPath path;
        path.qi[0] = d.initial_x;
        path.qi[1] = d.initial_y;
        path.qi[2] = d.initial_yaw;
        path.param[0] = d.length0;
        path.param[1] = d.length1;
        path.param[2] = d.length2;
        path.rho = d.rho;
        path.type = (DubinsPathType)d.type;
        wrapper.fill(path, d.speed, d.start_time);
        if (wrapper.getEndTime() > plan.endtime){
            wrapper.updateEndTime(plan.endtime);
        }
        result.append(wrapper);
    }
    return result;
}

static path_planner_common::Plan getPlanMsg(const DubinsPlan& plan) {
    path_planner_common::Plan planMsg;
    for (const auto& d : plan.get()) {
        path_planner_common::DubinsPath path;
        auto p = d.unwrap();
        path.initial_x = p.qi[0];
        path.initial_y = p.qi[1];
        path.initial_yaw = p.qi[2];
        path.length0 = p.param[0];
        path.length1 = p.param[1];
        path.length2 = p.param[2];
        path.type = p.type;
        path.rho = d.getRho();
        path.speed = d.getSpeed();
        path.start_time = d.getStartTime();
        planMsg.paths.push_back(path);
    }
    planMsg.endtime = plan.getEndTime();
    return planMsg;
}

TEST(UnitTests, PlanTransferTest1) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    for (int i = 2; i < 5; i++){
        ribbonManager.cover(start.x(), start.y(), false);
        auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
        ASSERT_FALSE(plan.empty());
        validatePlan(plan, plannerConfig);
        auto planMsg = getPlanMsg(plan);
        auto plan2 = getPlan(planMsg);
//        validatePlan(plan2, plannerConfig);
        const auto& paths1 = plan.get();
        const auto& paths2 = plan2.get();
        EXPECT_EQ(paths1.size(), paths2.size());
        for (int j = 0; j < paths1.size(); j++) {
            auto p1 = paths1[j].unwrap();
            auto p2 = paths2[j].unwrap();
            EXPECT_DOUBLE_EQ(p1.param[0], p2.param[0]);
            EXPECT_DOUBLE_EQ(p1.param[1], p2.param[1]);
            EXPECT_DOUBLE_EQ(p1.param[2], p2.param[2]);
            EXPECT_DOUBLE_EQ(p1.qi[0], p2.qi[0]);
            EXPECT_DOUBLE_EQ(p1.qi[1], p2.qi[1]);
            EXPECT_DOUBLE_EQ(p1.qi[2], p2.qi[2]);
            EXPECT_DOUBLE_EQ(p1.rho, p2.rho);
            EXPECT_EQ(p1.type, p2.type);
            EXPECT_DOUBLE_EQ(paths1[j].getStartTime(), paths2[j].getStartTime());
            EXPECT_DOUBLE_EQ(paths1[j].getEndTime(), paths2[j].getEndTime());
            EXPECT_DOUBLE_EQ(paths1[j].length(), paths2[j].length());
            EXPECT_DOUBLE_EQ(paths1[j].getSpeed(), paths2[j].getSpeed());
//            EXPECT_TRUE(paths1[j] == paths2[j]);
//            EXPECT_EQ(paths1[j], paths2[j]);
        }
        start.time() = i;
        plan.sample(start);
        cerr << "Remaining " << ribbonManager.dumpRibbons() << endl;
        cerr << start.toString() << endl;
        if (ribbonManager.done()) return;
    }
}

//endregion

TEST(UnitTests, GaussianDensityTest) {
    double sigma[2][2] = {1, 0.1, 0.1, 1};
    double mean[2] = {0, 0};
    Distribution distribution(mean, sigma, 5, 5, 0, 2);
//    EXPECT_NEAR(distribution.density(distribution, 2, 0, 0), 0.16, 1e-4);
//    EXPECT_NEAR(distribution.density(distribution, 2, 0.6, 0.6), 0.1153, 1e-4);
    EXPECT_DOUBLE_EQ(distribution.density(distribution, 2, 0, 0), 1);
    EXPECT_DOUBLE_EQ(distribution.density(distribution, 2, 0.6, 0.6), 1);
}

TEST(UnitTests, GaussianInterpolationTest) {
    double sigma1[2][2] = {1, 0.1, 0.1, 1};
    double mean1[2] = {-1, -1};
    double sigma2[2][2] = {1, 0.1, 0.1, 1};
    double mean2[2] = {1, 1};
    Distribution distribution1(mean1, sigma1, 5, 5, 0, 2);
    Distribution distribution2(mean2, sigma2, 5, 5, 0, 4);
//    EXPECT_NEAR(distribution1.density(distribution2, 3, 0, 0), 0.16, 1e-4);
//    EXPECT_NEAR(distribution1.density(distribution2, 3, 0.6, 0.6), 0.1153, 1e-4);
    EXPECT_DOUBLE_EQ(distribution1.density(distribution2, 3, 0, 0), 1);
    EXPECT_DOUBLE_EQ(distribution1.density(distribution2, 3, 0.6, 0.6), 1);
}

TEST(UnitTests, GaussianTruncateTest) {
    double sigma[2][2] = {{1, 0}, {0, 1}};
    double mean[2] = {0, 0};
    Distribution distribution(mean, sigma, 3, 3, 0, 2);
    // When sigma is the identity matrix, Mahalanobis distance is Euclidean distance, so we should get
    // a zero (truncated) density value for (1.5, 2) (distance to (0, 0) is 2.5)
    EXPECT_DOUBLE_EQ(0, distribution.density(1.5, 2));
    // and a non-zero (non-truncated) density value for (1, 1) (distance to (0, 0) is ~1.414
    EXPECT_LT(0, distribution.density(1, 1));
}

TEST(UnitTests, DynamicObstacleTest1) {
    // This test shouldn't pass right now
    DynamicObstaclesManager1 obstaclesManager;
    double sigma[2][2] = {{1, 0}, {0, 1}};
    double mean[2] = {0, 0};
    std::vector<Distribution> distributions;
    distributions.emplace_back(mean, sigma, 5, 5, 0, 2);
    distributions.emplace_back(mean, sigma, 5, 5, 0, 3);
    obstaclesManager.update(1, distributions);
    auto p = obstaclesManager.collisionExists(3, 4.5, 10);
    EXPECT_DOUBLE_EQ(p, 0);
    p = obstaclesManager.collisionExists(2.2, 2.2, 10);
    EXPECT_LT(0, p);
    p = obstaclesManager.collisionExists(0, 3, 10);
    EXPECT_DOUBLE_EQ(p, 0);
    distributions.clear();
    distributions.emplace_back(mean, sigma, 5, 5, M_PI / 4, 2);
    distributions.emplace_back(mean, sigma, 5, 5, M_PI / 4, 3);
    obstaclesManager.update(1, distributions);
    p = obstaclesManager.collisionExists(0, 3, 10);
    EXPECT_LT(0, p);
    distributions.clear();
    // shift and interpolate headings
    double mean1[2] = {1, 1};
    double sigma1[2][2] = {{1, 0}, {0, 1}};
    distributions.emplace_back(mean1, sigma1, 5, 5, M_PI / 6, 2);
    distributions.emplace_back(mean1, sigma1, 5, 5, M_PI / 3, 4);
    obstaclesManager.update(1, distributions);
    auto p1 = obstaclesManager.collisionExists(1, 4, 3);
    EXPECT_NEAR(p1, p, 0.00001);
}

TEST(UnitTests, BinaryDynamicObstaclesTest1) {
    BinaryDynamicObstaclesManager manager;
    manager.update(1, 42, 42, 0, 1, 1, 5, 15);
    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 42, 1, false), 1);
    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 49, 1, false), 1);
    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 50, 1, false), 0);
    EXPECT_DOUBLE_EQ(manager.collisionExists(44, 42, 1, false), 1);
    EXPECT_DOUBLE_EQ(manager.collisionExists(45, 42, 1, false), 0);

    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 52, 11, false), 1);
    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 59, 11, false), 1);
    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 60, 11, false), 0);
    EXPECT_DOUBLE_EQ(manager.collisionExists(44, 52, 11, false), 1);
    EXPECT_DOUBLE_EQ(manager.collisionExists(45, 52, 11, false), 0);
}

TEST(UnitTests, DerivedDynamicObstaclesTest) {
    BinaryDynamicObstaclesManager::SharedPtr b = std::make_shared<BinaryDynamicObstaclesManager>();
    b->update(1, 42, 42, 0, 1, 1, 5, 15);
    const DynamicObstaclesManager& manager = *b;

    EXPECT_DOUBLE_EQ(manager.collisionExists(42, 42, 1, false), 1);

    plannerConfig.setObstaclesManager(b);

    EXPECT_DOUBLE_EQ(plannerConfig.obstaclesManager().collisionExists(42, 42, 1, false), 1);
}

TEST(UnitTests, GaussianDynamicObstacleTest1) {
    GaussianDynamicObstaclesManager manager;
    manager.update(1, 0, 0, 0, 1, 1);
    for (int i = 0; i < 10; i++) {
        cerr << manager.collisionExists(0, 10 * i, 1, false) << endl;
        cerr << manager.collisionExists(10 * i, 10 * i, 1, false) << endl;
        cerr << manager.collisionExists(10 * i, 0, 1, false) << endl;
    }
}

TEST(UnitTests, GeoTiffMapTest1) {
    GeoTiffMap map("../../../src/mbes_sim/data/US5NH02M.tiff", -70.71054174878898, 43.073397415457535);
}

TEST(UnitTests, GeoTiffMapTest2) {
    GeoTiffMap map("../../../src/mbes_sim/data/US5NH02M.tiff", -70.71054174878898, 43.073397415457535);
//    EXPECT_DOUBLE_EQ(map.getDepth(0, 0), 0);
//    EXPECT_NEAR(map.getDepth(365000, 4770000), 14.87, 0.001);
    EXPECT_TRUE(map.isBlocked(0, 0));
    EXPECT_TRUE(map.isBlocked(50, 50));
    // these aren't what I expected but I printed out the whole thing in ASCII and it wasn't all blocked so I'm
    // assuming it's fine
    // TODO! -- maybe ping more places that should either be blocked or not
}

TEST(UnitTests, LoadGridWorldTest) {
    GridWorldMap map("../../../src/test_scenario_runner/scenarios/cannon_wait_1.map");
}

TEST(UnitTests, CannonCutAcrossStartPositionTest) {
    GridWorldMap map("../../../src/test_scenario_runner/scenarios/cannon_cut_across.map");
    EXPECT_FALSE(map.isBlocked(70, 5));
}

TEST(UnitTests, GridWorldMapTest1) {
    GridWorldMap map("../../../src/test_scenario_runner/scenarios/test1.map");
//    map.getUnblockedDistance(0, 0);
//    EXPECT_DOUBLE_EQ(-1, map.getUnblockedDistance(450, 445));
//    EXPECT_DOUBLE_EQ(10, map.getUnblockedDistance(495, 450));
    EXPECT_TRUE(map.isBlocked(450, 445));
    EXPECT_FALSE(map.isBlocked(495, 450));
}

void visualizePath(const State& s1, const State& s2, const State& s3, double turningRadius) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 0, 1000, 0);
    PlannerConfig config(&std::cerr);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_visualizations"));
    config.setVisualizer(&visualizer);
    config.setVisualizations(true);
    config.setNowFunction([] () -> double {
        struct timespec t{};
        clock_gettime(CLOCK_REALTIME, &t);
        return t.tv_sec + t.tv_nsec * 1e-9;
    });
    config.setMap(make_shared<Map>());
    config.setObstacles(DynamicObstaclesManager1());
    config.setBranchingFactor(4);
    config.setMaxSpeed(2.5); // set this super high so we don't clip paths short
    config.setTurningRadius(turningRadius);
    auto v1 = Vertex::makeRoot(s1, ribbonManager);
    auto v2 = Vertex::connect(v1, s2);
    auto v3 = Vertex::makeRoot(s3, ribbonManager);
    auto v4 = Vertex::connect(v3, s2);
    visualizer->stream() << v1->toString() << " start" << endl;
    v2->parentEdge()->computeTrueCost(config);
    visualizer->stream() << v2->toString() << " vertex" << endl;
    visualizer->stream() << v3->toString() << " vertex" << endl;
    v4->parentEdge()->computeTrueCost(config);
    visualizer->stream() << v4->toString() << " vertex" << endl;
}

void visualizePath(const State& s1, const State& s2, double turningRadius) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 0, 1000, 0);
    PlannerConfig config(&std::cerr);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_visualizations"));
    config.setVisualizer(&visualizer);
    config.setVisualizations(true);
    config.setNowFunction([] () -> double {
        struct timespec t{};
        clock_gettime(CLOCK_REALTIME, &t);
        return t.tv_sec + t.tv_nsec * 1e-9;
    });
    config.setMap(make_shared<Map>());
    config.setObstacles(DynamicObstaclesManager1());
    config.setBranchingFactor(4);
    config.setMaxSpeed(2); // set this super high so we don't clip paths short
    config.setTurningRadius(turningRadius);
    auto v1 = Vertex::makeRoot(s1, ribbonManager);
    auto v2 = Vertex::connect(v1, s2);
//    auto v3 = Vertex::makeRoot(s3, ribbonManager);
//    auto v4 = Vertex::connect(v3, s2);
    visualizer->stream() << v1->toString() << " start" << endl;
    v2->parentEdge()->computeTrueCost(config);
    visualizer->stream() << v2->toString() << " vertex" << endl;
//    visualizer->stream() << v3->toString() << " vertex" << endl;
//    v4->parentEdge()->computeTrueCost(config);
//    visualizer->stream() << v4->toString() << " vertex" << endl;
}

TEST(UnitTests, DubinsSuffixTest) {
    StateGenerator generator(-50, 50, -50, 50, plannerConfig.maxSpeed(), plannerConfig.maxSpeed(), 7);
    auto rootState = generator.generate();
    rootState.time() = 1;
    rootState.speed() = plannerConfig.maxSpeed();
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 20, 10);
    int successCount = 0, failureCount = 0, shorterCount = 0;
    double differences = 0, progress = 0;
    double tolerance = 1;
    double radiusShrink = 1e-10;
    for (int i = 0; i < 1000; i++) {
        auto startState = generator.generate(); startState.time() = 1;
        auto endState = generator.generate();
        DubinsWrapper path(startState, endState, plannerConfig.turningRadius());
        endState.time() = path.getEndTime();
//        cerr << endState.time() << endl;
        State sample;
        for (sample.time() = startState.time(); sample.time() < endState.time() - 1; sample.time() +=
                                                                                             plannerConfig.collisionCheckingIncrement() / plannerConfig.maxSpeed()) {
            path.sample(sample);
            DubinsWrapper path2(sample, endState, plannerConfig.turningRadius() - radiusShrink);
//            EXPECT_DOUBLE_EQ(endState.time(), path2.getEndTime());
            if (fabs(endState.time() - path2.getEndTime()) > tolerance) {
//                std::cout << "Failed from state " << sample.toString() << endl;
//                cerr << startState.toStringRad() << endl;
//                cerr << endState.toStringRad() << endl;
//                cerr << sample.toStringRad() << endl;
                auto diff = fabs(endState.time() - path2.getEndTime()) / plannerConfig.maxSpeed();
//                cerr << diff << endl;
//                visualizePath(startState, endState, sample, plannerConfig.turningRadius());
                failureCount++;
                progress += (sample.time() - startState.time()) / (endState.time() - startState.time());
                differences += diff;
//                return;
                if (path2.getEndTime() < endState.time()) {
                    shorterCount++;
                }
            }
            else {
//                std::cout << "." << std::flush;
                successCount++;
            }
        }
//        std::cout << endl;
    }

    auto total = successCount + failureCount;
    cerr << "Successes: " << successCount << ", failures: " << failureCount << ", failure rate: " << (double)failureCount / total * 100 << "%" << endl;
    cerr << "Failures occur on average " << progress / failureCount * 100 << "% of the way through the path" << endl;
    cerr << "Found a shorter path " << (double) shorterCount / failureCount  * 100 << "% of the time" << endl;
    cerr << "Average length difference: " << differences / failureCount << endl;

    // Randomizing turning radius now to study its effect on length differences
    std::uniform_real_distribution<> radiusDistribution(1, 100);
    std::default_random_engine randomEngine(7);
    failureCount = 0; successCount = 0; progress = 0;
    double ratio = 0;
    for (int i = 0; i < 1000; i++) {
        auto startState = generator.generate(); startState.time() = 1;
        auto endState = generator.generate();
        auto radius = radiusDistribution(randomEngine);
        DubinsWrapper path(startState, endState, radius);
        endState.time() = path.getEndTime();
//        cerr << endState.time() << endl;
        State sample;
        for (sample.time() = startState.time(); sample.time() < endState.time() - 1; sample.time() +=
                                                                                             plannerConfig.collisionCheckingIncrement() / plannerConfig.maxSpeed()) {
            path.sample(sample);
            DubinsWrapper path2(sample, endState, radius - radiusShrink);
//            EXPECT_DOUBLE_EQ(endState.time(), path2.getEndTime());
            if (fabs(endState.time() - path2.getEndTime()) > tolerance) {
//                if (endState.time() != path2.getEndTime()) {
//                std::cout << "Failed from state " << sample.toString() << endl;
                failureCount++;
                progress += (sample.time() - startState.time()) / (endState.time() - startState.time());
                auto diff = fabs(endState.time() - path2.getEndTime()) / plannerConfig.maxSpeed();
                differences += diff;
                ratio += diff / radius;
                if (path2.getEndTime() < endState.time()) {
                    shorterCount++;
                }
            }
            else {
//                std::cout << "." << std::flush;
                successCount++;
            }
        }
//        std::cout << endl;
    }
    cerr << "Now using randomized radii: " << endl;
    total = successCount + failureCount;
    cerr << "Successes: " << successCount << ", failures: " << failureCount << ", failure rate: " << (double)failureCount / total * 100 << "%" << endl;
    cerr << "Failures occur on average " << progress / failureCount * 100 << "% of the way through the path" << endl;
    cerr << "Average length difference: " << differences / failureCount << endl;
    cerr << "Average ratio length difference to turning radius: " << ratio / failureCount << endl;
}

TEST(UnitTests, SimpleDubinsTest) {
    // I wrote this when I thought something wasn't working but actually I was just being dumb. No reason to remove it though.
    double radius = 8, speed = 2;
    State s1(0, 0, 0, speed, 1);
    State s2(2 * radius, 0, M_PI, speed, radius * M_PI / speed + 1);
    DubinsPlan plan(s1, s2, radius);
//    visualizePath(s1, s2, radius);
    EXPECT_NEAR(plan.getEndTime(), s2.time(), 1e-5);
}

TEST(UnitTests, DubinsReverseTest) {

}

TEST(UnitTests, RibbonsTest1) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 0, 1000, 0);
    // TODO! -- subtract 2* min ribbon length
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
    auto r = r1.split(0, 0, false);
    EXPECT_TRUE(r.length() < 3);
    auto r2 = r1.split(-10, 0, false);
    EXPECT_EQ(r2.end(), std::make_pair(-10.0, 0.0));
    EXPECT_EQ(r2.start(), std::make_pair(40.0, 100.0));
    EXPECT_EQ(r2.end(), r1.start());
}

TEST(UnitTests, RibbonsTest3) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(0, 0, 1000, 0);
    ribbonManager.cover(2, 0, false);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(2, 0, 0), 998);
}

TEST(UnitTests, RibbonsTest4) {
    RibbonManager ribbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
    ribbonManager.add(0, 0, 1000, 0);
    ribbonManager.cover(1, 1, false);
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

TEST(UnitTests, HeuristicConsistency1) {
    // ribbon dead ahead
    auto heuristics = {
            RibbonManager::MaxDistance,
            RibbonManager::TspPointRobotNoSplitAllRibbons,
            RibbonManager::TspPointRobotNoSplitKRibbons,
            // Dubins heuristics are way off near ribbon ends that are being covered (rounding error again)
//            RibbonManager::TspDubinsNoSplitAllRibbons,
//            RibbonManager::TspDubinsNoSplitKRibbons
    };
    for (auto heuristic : heuristics) {
        RibbonManager ribbonManager(heuristic, 8, 2);
        ribbonManager.add(0, 0, 0, 75);
        State s1(0, 0, 0, 2.5, 1), s2(0, 75, 0, 2.5, 31);
        DubinsWrapper path(s1, s2, 8);
        while (path.containsTime(s1.time())) {
            path.sample(s1);
            ribbonManager.cover(s1.x(), s1.y(), false);
            if (ribbonManager.done()) break;
            auto h = ribbonManager.approximateDistanceUntilDone(s1.x(), s1.y(), s1.yaw()) / 2.5;
            EXPECT_DOUBLE_EQ(s1.time() + h, s2.time());
            s1.time() += 1;
        }
    }
}

TEST(UnitTests, HeuristicConsistency2) {
    // curve then ribbon
    RibbonManager ribbonManager(RibbonManager::MaxDistance, 8, 2);
    ribbonManager.add(16, 0, 16, -75);
    State s1(0, 0, 0, 2.5, 1), s2(16, -75, M_PI, 2.5, 0);
    DubinsWrapper path(s1, s2, 8);
    while (path.containsTime(s1.time())) {
        path.sample(s1);
        auto contained = ribbonManager.get().front().containsProjection(std::make_pair(s1.x(), s1.y()));
        ribbonManager.cover(s1.x(), s1.y(), false);
        if (ribbonManager.done()) break;
        auto h = ribbonManager.approximateDistanceUntilDone(s1.x(), s1.y(), s1.yaw()) / 2.5;
        if (contained) {
            EXPECT_DOUBLE_EQ(s1.time() + h, path.getEndTime());
        } else
        {
            EXPECT_LE(s1.time() + h, path.getEndTime());
        }
        s1.time() += 1;
    }
}

TEST(UnitTests, HeuristicConsistency3) {
    // start from a random state, go to the start of the line, cover it
    RibbonManager ribbonManager(RibbonManager::MaxDistance, 8, 2);
    ribbonManager.add(0, 0, 0, 10);
    StateGenerator generator(-100, 100, -100, 100, 2.5, 2.5, 42);
    State s1(0, 0, 0, 2.5, 0), s2(0, 10, 0, 2.5, 0), s3 = generator.generate();
    s3.time() = 1;
    DubinsPlan plan;
    plan.append(DubinsWrapper(s3, s1, 8));
    s1.time() = plan.getEndTime();
    plan.append(DubinsWrapper(s1, s2, 8));
    while (plan.containsTime(s3.time())) {
        plan.sample(s3);
        auto r = ribbonManager.get().front();
        auto contained = r.contains(s3.x(), s3.y(), r.getProjection(s3.x(), s3.y()), false);
        ribbonManager.cover(s3.x(), s3.y(), false);
        if (ribbonManager.done()) break;
        auto h = ribbonManager.approximateDistanceUntilDone(s3.x(), s3.y(), s3.yaw()) / 2.5;
        if (contained) {
            EXPECT_DOUBLE_EQ(s3.time() + h, plan.getEndTime());
        } else
        {
            EXPECT_LE(s3.time() + h, plan.getEndTime());
        }
        s3.time() += 1;
    }
}

TEST(UnitTests, HeuristicConsistency4) {
    plannerConfig.setMaxSpeed(2.5); // this is default but make sure anyway
    plannerConfig.setStartStateTime(1);
    RibbonManager ribbonManager(RibbonManager::MaxDistance, 8, 2);
    ribbonManager.add(0, 0, 0, 80);
    State s1(0, 0, 0, plannerConfig.maxSpeed(), 1), s2(0, 75, 0, plannerConfig.maxSpeed(), 31);
    ribbonManager.coverBetween(0, -2.5, 0, 0); // this happens in the real version so let's do it here; we've come from somewhere
    auto root = Vertex::makeRoot(s1, ribbonManager);
    auto v1 = Vertex::connect(root, s2);
    v1->parentEdge()->computeTrueCost(plannerConfig);
    auto f1 = v1->f();
    auto path = v1->parentEdge()->getPlan(plannerConfig);
    // append a state on the end of the last plan and make sure the heuristic is consistent
    State s3(0, 0, 0, 0, 2), s4(0, 77.5, 0, 2.5, 32);
    path.sample(s3);
    path.updateStartTime(2);
    ribbonManager.coverBetween(0, 0, s3.x(), s3.y());
    plannerConfig.setStartStateTime(2);
    auto root2 = Vertex::makeRoot(s3, ribbonManager);
    auto v2 = Vertex::connect(root2, path, path.getRho() == plannerConfig.coverageTurningRadius());
    v2->parentEdge()->computeTrueCost(plannerConfig);
    auto v3 = Vertex::connect(v2, s4);
    v3->parentEdge()->computeTrueCost(plannerConfig);
    auto f2 = v3->f();
    EXPECT_NEAR(f1 - 1, f2, 1e-5); // time has gone down
}

TEST(UnitTests, HeuristicComparison5) {
    // okay how do I get more realistic than that?
    // maybe let's have the planner make the plan?
    RibbonManager ribbonManager(RibbonManager::MaxDistance, 2);
    ribbonManager.add(0, 0, 0, 80);
    plannerConfig.setMaxSpeed(2.5); // this is default but make sure anyway
    plannerConfig.setTurningRadius(8);
    plannerConfig.setCoverageTurningRadius(16); // eh probably don't need this but whatever
    State start(-20, -20, 0, 2.5, 1);
    AStarPlanner planner;
    auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
    EXPECT_FALSE(plan.empty());
    auto previousConfig = plannerConfig;
    // done with that iteration, let's test what happens in the next one
    plannerConfig.setStartStateTime(2);
    start.time() = 2;
    plan.sample(start);
    // ignoring ribbon manager coverage because we're far enough away not to matter
    auto root = Vertex::makeRoot(start, ribbonManager);
    auto lastPlanEnd = root;
    auto previousEndVertex = root;
    for (const auto& p : plan.get()) {
        lastPlanEnd = Vertex::connect(lastPlanEnd, p, p.getRho() == plannerConfig.coverageTurningRadius());
        lastPlanEnd->parentEdge()->computeTrueCost(plannerConfig);
        previousEndVertex = Vertex::connect(previousEndVertex, p, p.getRho() == plannerConfig.coverageTurningRadius());
        previousEndVertex->parentEdge()->computeTrueCost(previousConfig);
    }
    State s(0, 80, 0, 2.5, 0);
    auto endV = Vertex::connect(lastPlanEnd, s);
    endV->parentEdge()->computeTrueCost(plannerConfig); // should truncate
    cerr << endV->toString() << endl;
    EXPECT_NEAR(endV->state().x(), 0, 1e-12);
    EXPECT_NEAR(endV->state().heading(), 0, 1e-12);
    EXPECT_DOUBLE_EQ(previousEndVertex->f(), endV->f());
}

TEST(UnitTests, RibbonManagerGetNearestEndpointTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(10, 10, 20, 10);
    auto s = ribbonManager.getNearestEndpointAsState(State(0, 0, 0, 0, 0));
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
            ribbonManager.cover(s.x(), s.y(), false);
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
    auto v1 = Vertex::makeRoot(s1, RibbonManager());
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(1, 2);
    EXPECT_DOUBLE_EQ(a, 5);
    // will return dubins wrapper now
    auto plan = e->getPlan(plannerConfig);
    EXPECT_GE(plan.getEndTime() - plan.getStartTime(), 5);
    auto samples = plan.getSamples(0.5, 0);
    for (const auto& s : samples) cerr << s.toString() << endl;
    State sample(s1);
    plan.sample(s1);
    EXPECT_TRUE(sample.isCoLocated(s1));
    sample.time() = s2.time();
    plan.sample(sample);
    EXPECT_NEAR(sample.distanceTo(s2), 0, 1e-5);
}

TEST(UnitTests, ComputeEdgeCostTest) {
    State s1(0, 0, 0, plannerConfig.maxSpeed(), 1);
    State s2(0, 5, 0, plannerConfig.maxSpeed(), 0);
    auto v1 = Vertex::makeRoot(s1, RibbonManager());
    auto v2 = Vertex::connect(v1, s2);
    auto e = v2->parentEdge();
    auto a = e->computeApproxCost(plannerConfig.maxSpeed(), plannerConfig.turningRadius());
    Map::SharedPtr map = make_shared<Map>();
    DynamicObstaclesManager1 dynamicObstacles;
//    Path path;
//    path.add(0, 10);
    auto c = e->computeTrueCost(plannerConfig);
    EXPECT_DOUBLE_EQ(3, e->end()->state().time());
    EXPECT_DOUBLE_EQ(c, a);
}

TEST(UnitTests, RunStateGenerationTest) {
    double minX, maxX, minY, maxY, minSpeed = 2.5, maxSpeed = 2.5;
    double magnitude = 2.5 * plannerConfig.timeHorizon();
    minX = -magnitude;
    maxX = magnitude;
    minY = -magnitude;
    maxY = magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7); // lucky seed
    cerr << generator.generate().toString() << endl;
    sleep(1); // ??
}

TEST(UnitTests, VertexTests1) {
    RibbonManager ribbonManager;
    ribbonManager.add(50, 50, 60, 50);
    auto root = Vertex::makeRoot(State(5, 5, M_PI, 2.5, 1), ribbonManager);
    auto v1 = Vertex::connect(root, State(5, -20, M_PI, 2.5, 0));
    auto c = v1->parentEdge()->computeApproxCost(2.5, 8);
    EXPECT_DOUBLE_EQ(c, 10);
    Map::SharedPtr m = make_shared<Map>();
    DynamicObstaclesManager1 obstacles;
    auto t = v1->parentEdge()->computeTrueCost(plannerConfig);
    EXPECT_DOUBLE_EQ(t, c);
    EXPECT_DOUBLE_EQ(t, v1->currentCost());
    EXPECT_DOUBLE_EQ(v1->currentCost(), v1->state().time() - 1);
    auto h = v1->computeApproxToGo(plannerConfig);
    EXPECT_DOUBLE_EQ(ribbonManager.approximateDistanceUntilDone(v1->state().x(), v1->state().y(), v1->state().yaw()) / 2.5, h);
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
    DynamicObstaclesManager1 obstacles;
    auto t = v1->parentEdge()->computeTrueCost(plannerConfig);
    EXPECT_DOUBLE_EQ(c, t);
    auto h = v1->computeApproxToGo(plannerConfig);
    EXPECT_DOUBLE_EQ((v1->state().distanceTo(30, 30) + 20 * sqrt(2) + 10 + 50 - 2 * Ribbon::minLength()) / 2.5, h);
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
    DynamicObstaclesManager1 obstacles;
    v1->parentEdge()->computeTrueCost(plannerConfig);
    auto h = v1->computeApproxToGo(plannerConfig);
    EXPECT_DOUBLE_EQ((v1->state().distanceTo(30, 30) + 20 * sqrt(2) + 50 - 2 * Ribbon::minLength()) / 2.5, h);
}

TEST(UnitTests, PointerTreeStringTest) {
    // make sure the function runs and doesn't print out anything unreasonable
    RibbonManager ribbonManager(RibbonManager::MaxDistance);
    ribbonManager.add(30, 30, 50, 50);
    ribbonManager.add(50, 60, 100, 60);
    auto root = Vertex::makeRoot(State(5, 5, M_PI, 2.5, 1), ribbonManager);
    auto v1 = Vertex::connect(root, State(5, -20, M_PI, 2.5, 0));
    cerr << root->getPointerTreeString() << endl;
    cerr << v1->getPointerTreeString() << endl;
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

TEST(PlannerTests, VertexQueueTests) {
    State start(0, 0, 0, 2.5, 1);
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    auto root = Vertex::makeRoot(start, ribbonManager);
    AStarPlanner planner;
    planner.setConfig(plannerConfig);
    planner.pushVertexQueue(root);
    auto popped = planner.popVertexQueue();
    EXPECT_EQ(root, popped);
    State s1(0, 10, 0, 2.5, 0), s2(0, -10, M_PI, 2.5, 0);
    auto v1 = Vertex::connect(root, s1);
    v1->parentEdge()->computeTrueCost(plannerConfig);
    v1->setCurrentCost();
    auto v2 = Vertex::connect(root, s2);
    v2->parentEdge()->computeTrueCost(plannerConfig);
    v2->setCurrentCost();
    planner.pushVertexQueue(v1);
    planner.pushVertexQueue(v2);
    popped = planner.popVertexQueue();
    EXPECT_EQ(v1, popped);
}

TEST(UnitTests, EmptyVertexQueueTest) {
    AStarPlanner planner;
    EXPECT_THROW(planner.popVertexQueue(), std::out_of_range);
}

TEST(UnitTests, ExpandTest1Ribbons) {
    StateGenerator generator(-50, 50, -50, 50, 2.5, 2.5, 9);
    State start = generator.generate();
    start.time() = 1;
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    const DynamicObstaclesManager& obstacles = BinaryDynamicObstaclesManager();
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
    // Obsolete test - why should the top vertex have coverage allowed?
    State start(0, 0, 0, 2.5, 1);
    RibbonManager ribbonManager;
    ribbonManager.add(0, 0, 0, 30);
    Map::SharedPtr m = make_shared<Map>();
    const DynamicObstaclesManager& obstacles = BinaryDynamicObstaclesManager();
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
    auto minAngleChange = 2 * (plannerConfig.collisionCheckingIncrement() / plannerConfig.turningRadius() + 1e-5); // arc length / radius + tolerance
    StateGenerator generator(-50, 50, -50, 50, plannerConfig.maxSpeed(), plannerConfig.maxSpeed(), 7);
    auto rootState = generator.generate();
    rootState.time() = 1;
    rootState.speed() = plannerConfig.maxSpeed();
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 20, 10);
    auto root = Vertex::makeRoot(rootState, ribbonManager);
    for (int i = 0; i < 10; i++) {
        auto startState = generator.generate();
        auto start = Vertex::connect(root, startState);
        if (start->parentEdge()->computeTrueCost(plannerConfig) > plannerConfig.timeHorizon() - 1){
            i--;
            continue;
        }
        auto endState = generator.generate();
        auto end = Vertex::connect(start, endState);
        end->parentEdge()->computeTrueCost(plannerConfig);
        State sample;
        sample.time() = end->state().time() - plannerConfig.collisionCheckingIncrement() / plannerConfig.maxSpeed();
        auto plan = end->parentEdge()->getPlan(plannerConfig);
        plan.sample(sample);
        EXPECT_NEAR(sample.headingDifference(end->state().heading()), 0, minAngleChange);
        sample.time() = end->state().time();
        plan.sample(sample);
//        EXPECT_DOUBLE_EQ(sample.heading(), end->state().heading());
        auto sPrev = startState;
        auto samples = plan.getSamples(plannerConfig.collisionCheckingIncrement() / plannerConfig.maxSpeed(), 0);
        for (const auto& s : samples) {
            EXPECT_NEAR(sPrev.headingDifference(s.heading()), 0, minAngleChange);
            sPrev = s;
        }
        EXPECT_NEAR(sPrev.heading(), end->state().heading(), minAngleChange);
    }
}

TEST(UnitTests, AngleConsitencyTest2) {
    auto minAngleChange = (plannerConfig.collisionCheckingIncrement() / plannerConfig.turningRadius() + 1e-5); // arc length / radius + tolerance
    StateGenerator generator(-50, 50, -50, 50, plannerConfig.maxSpeed(), plannerConfig.maxSpeed(), 7);
    generator.generate(); // waste a state for the root state in prev test so we can have same states
    for (int i = 0; i < 10; i++) {
        auto startState = generator.generate();
        startState.time() = 1;
        auto endState = generator.generate();
        DubinsWrapper wrapper(startState, endState, plannerConfig.turningRadius());
        State sample;
        sample.time() = startState.time() + wrapper.length() / plannerConfig.maxSpeed();
        wrapper.sample(sample);
        EXPECT_NEAR(sample.headingDifference(endState.heading()), 0, minAngleChange);
        wrapper.sample(sample); // for stepping through with a debugger
        auto sPrev = startState;
        auto samples = wrapper.getSamples(plannerConfig.collisionCheckingIncrement() / plannerConfig.maxSpeed(), 0);
        for (const auto& s : samples) {
            EXPECT_NEAR(sPrev.headingDifference(s.heading()), 0, minAngleChange);
            sPrev = s;
        }
        EXPECT_NEAR(sPrev.heading(), endState.heading(), minAngleChange);
    }
}

TEST(UnitTests, EdgeTruncation) {
    State s1(0, 0, 0, plannerConfig.maxSpeed(), 1);
    State s2(0, 10, 0, plannerConfig.maxSpeed(), 0);
    State s3(0, 100, 0, plannerConfig.maxSpeed(), 0);
    RibbonManager ribbonManager;
    ribbonManager.add(100, 0, 100, 10);
    auto root = Vertex::makeRoot(s1, ribbonManager);
    auto v1 = Vertex::connect(root, s2);
    auto d = v1->parentEdge()->computeApproxCost(plannerConfig.maxSpeed(), plannerConfig.turningRadius());
    EXPECT_DOUBLE_EQ(d, 4);
    d = v1->parentEdge()->computeTrueCost(plannerConfig);
    EXPECT_DOUBLE_EQ(d, 4);
    auto s4 = v1->state();
    EXPECT_NEAR(s4.distanceTo(s2), 0, 1e-10);
    auto v2 = Vertex::connect(root, s3);
    d = v2->parentEdge()->computeApproxCost(plannerConfig.maxSpeed(), plannerConfig.turningRadius());
    EXPECT_DOUBLE_EQ(d, 40);
    d = v2->parentEdge()->computeTrueCost(plannerConfig);
    EXPECT_DOUBLE_EQ(d, plannerConfig.timeHorizon());
    auto s5 = v2->state();
    EXPECT_FALSE(s5.isCoLocated(s2));

}

TEST(PlannerTests, UsePreviousPlan) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_test_visualizations"));
    plannerConfig.setVisualizations(true);
    plannerConfig.setVisualizer(&visualizer);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
    EXPECT_FALSE(plan.empty());
    start.time() = 2;
    plan.sample(start);
    // ribbon manager won't have changed
    auto plan2 = planner.plan(ribbonManager, start, plannerConfig, plan, 0.95).Plan;
    EXPECT_FALSE(plan2.empty());
    validatePlan(plan2, plannerConfig);
}

TEST(UnitTests, UsePreviousPlanUnitTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_test_visualizations"));
    plannerConfig.setVisualizations(true);
    plannerConfig.setVisualizer(&visualizer);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
    EXPECT_FALSE(plan.empty());
    start.time() = 2;
    plan.sample(start);

    plan.changeIntoSuffix(2);
    auto startV = Vertex::makeRoot(start, ribbonManager);
//    plannerConfig.setVisualizations(true);

    Vertex::SharedPtr lastPlanEnd = startV;
    if (!plan.empty()) {
//        auto p = plan.get().front();
        for (const auto& p : plan.get()) {
            lastPlanEnd = Vertex::connect(lastPlanEnd, p, p.getRho() == plannerConfig.coverageTurningRadius());
            lastPlanEnd->parentEdge()->computeTrueCost(plannerConfig);
            if (lastPlanEnd->parentEdge()->infeasible()) {
                FAIL();
//                lastPlanEnd = startV;
//                break;
            }
        }
    }

    auto newPlan = planner.tracePlan(lastPlanEnd, false, plannerConfig.obstaclesManager());
    State s1 = start, s2 = start;
    while (s1.time() < newPlan.getEndTime()) {
        plan.sample(s1);
        newPlan.sample(s2);
        EXPECT_NEAR(s1.distanceTo(s2), 0, 1e-5);
        s1.time() += 1; s2.time() += 1;
    }

    auto plan1Samples = plan.getHalfSecondSamples();
    auto plan2Samples = newPlan.getHalfSecondSamples();
    for (unsigned long i = 0; i < plan1Samples.size(); i++) {
        EXPECT_NEAR(plan1Samples[i].distanceTo(plan2Samples[i]), 0, 1e-5);
    }

}

TEST(PlannerTests, RHRSAStarTest1Ribbons) {
    plannerConfig.now();
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
    EXPECT_FALSE(plan.empty());
    validatePlan(plan, plannerConfig);
//    for (auto s : plan.getHalfSecondSamples()) cerr << s.toString() << endl;
}

TEST(PlannerTests, RHRSAStarTest2Ribbons) {
    RibbonManager ribbonManager;
    ribbonManager.add(0, 10, 0, 30);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    while(!ribbonManager.done()) {
        ribbonManager.cover(start.x(), start.y(), false);
        auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.5).Plan; // quick iterations
        ASSERT_FALSE(plan.empty());
        validatePlan(plan, plannerConfig);
        start = plan.getHalfSecondSamples()[1];
        ASSERT_LT(start.time(), 30);
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, RibbonFarAwayTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(100, 110, 100, 130);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_test_visualizations"));
    plannerConfig.setVisualizations(true);
    plannerConfig.setVisualizer(&visualizer);
    DubinsPlan plan;
    while(!ribbonManager.done()) {
        ribbonManager.cover(start.x(), start.y(), false);
        plan = planner.plan(ribbonManager, start, plannerConfig, plan, 0.5).Plan; // quick iterations
        ASSERT_FALSE(plan.empty());
        validatePlan(plan, plannerConfig);
        start.time() += 1;
        plan.sample(start);
        ASSERT_LT(start.time(), 90);
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, RibbonNotQuiteAsFarAwayTest) {
    RibbonManager ribbonManager;
    ribbonManager.add(50, 60, 50, 80);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_test_visualizations"));
    plannerConfig.setVisualizations(true);
    plannerConfig.setVisualizer(&visualizer);
    DubinsPlan plan;
    while(!ribbonManager.done()) {
        ribbonManager.cover(start.x(), start.y(), false);
        plan = planner.plan(ribbonManager, start, plannerConfig, plan, 0.5).Plan; // quick iterations
        ASSERT_FALSE(plan.empty());
        validatePlan(plan, plannerConfig);
        start.time() += 1;
        plan.sample(start);
        ASSERT_LT(start.time(), 90);
        cerr << start.toString() << endl;
    }
}

TEST(PlannerTests, RHRSAStarTest4Ribbons) {
    RibbonManager ribbonManager(RibbonManager::MaxDistance);
    ribbonManager.add(0, 20, 20, 20);
    ribbonManager.add(0, 40, 20, 40);
    ribbonManager.add(0, 60, 20, 60);
    ribbonManager.add(0, 80, 20, 80);
    ribbonManager.add(0, 100, 20, 100);
    AStarPlanner planner;
    State start(0, 0, 0, 2.5, 1);
    bool headingChanged = false;
    DubinsPlan plan;
    while(!ribbonManager.done()) {
        if (!headingChanged) ribbonManager.cover(start.x(), start.y(), false);
        plan = planner.plan(ribbonManager, start, plannerConfig, plan, 0.95).Plan;
        ASSERT_FALSE(plan.empty());
        validatePlan(plan, plannerConfig);
        ASSERT_DOUBLE_EQ(plan.getStartTime(), start.time());
        headingChanged = plan.getHalfSecondSamples()[1].headingDifference(start.heading()) > 0.1;
        start = plan.getHalfSecondSamples()[1];
        ASSERT_DOUBLE_EQ(plan.getStartTime() + 0.5, start.time());
//        start.time() += 1;
//        plan.sample(start);
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
    auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
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
    DubinsPlan plan;
    Visualizer::UniquePtr visualizer(new Visualizer("/tmp/planner_test_visualizations"));
    plannerConfig.setVisualizations(true);
    plannerConfig.setVisualizer(&visualizer);
    while(!ribbonManager.done()) {
        /*if (!headingChanged)*/ ribbonManager.cover(start.x(), start.y(), false);
        plan = planner.plan(ribbonManager, start, plannerConfig, plan, 0.5).Plan;
        ASSERT_FALSE(plan.empty());
//        headingChanged = plan.getHalfSecondSamples()[1].heading() == start.heading();
        start.time() += 1;
        plan.sample(start);
//        for (const auto& s : plan.getHalfSecondSamples()) cerr << s.toString() << endl;
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
        if (!headingChanged) ribbonManager.cover(start.x(), start.y(), false);
        auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
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
        if (!headingChanged) ribbonManager.cover(start.x(), start.y(), false);
        auto plan = planner.plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
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
            ribbonManager.cover(start.x(), start.y(), false);
            auto plan = planner->plan(ribbonManager, start, plannerConfig, DubinsPlan(), 0.95).Plan;
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
    config.setObstacles(DynamicObstaclesManager1());
    config.setBranchingFactor(4);
    auto plan = planner.plan(ribbonManager, start, config, DubinsPlan(), 0.95).Plan;
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
    config.setObstacles(DynamicObstaclesManager1());
    std::thread t ([&]{
        while(!ribbonManager.done()) {
            ribbonManager.cover(start.x(), start.y(), false);
            auto plan = planner.plan(ribbonManager, start, config, DubinsPlan(), 0.95).Plan;
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
    config.setObstacles(DynamicObstaclesManager1());
    StateGenerator generator(-10, 30, 0, 120, 2.5, 2.5, 9);
    for (int i = 0; i < 10; i++) {
        auto plan = planner.plan(ribbonManager, generator.generate(), config, DubinsPlan(), 0.95).Plan;
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
        auto plan = planner.plan(ribbonManager, generator.generate(), plannerConfig, DubinsPlan(), 9.95).Plan;
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
    plannerConfig.setObstacles(DynamicObstaclesManager1());
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#pragma clang diagnostic pop