#include "executive.h"
#include "../planner/SamplingBasedPlanner.h"
#include "../planner/AStarPlanner.h"
#include "../planner/PotentialFieldPlanner.h"
#include "../common/map/GeoTiffMap.h"
#include "../common/map/GridWorldMap.h"
#include <path_planner_common/State.h>
#include "../trajectory_publisher.h"
#include "../planner/search/Edge.h"

#include <unistd.h>
#include <string>
#include <thread>
#include <fstream>
#include <wait.h>
#include <future>
#include <condition_variable>


using namespace std;

Executive::Executive(TrajectoryPublisher *trajectoryPublisher)
{
    m_TrajectoryPublisher = trajectoryPublisher;
    m_PlannerConfig.setNowFunction([&] { return m_TrajectoryPublisher->getTime(); });
}

Executive::~Executive() {
    terminate();
    m_PlanningFuture.wait_for(chrono::seconds(2));
}

double Executive::getCurrentTime()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
    if ((m_LastHeading - heading) / m_LastUpdateTime <= c_CoverageHeadingRateMax) {
        std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
        m_RibbonManager.cover(x, y, false);
    }
    m_LastUpdateTime = t; m_LastHeading = heading;
    m_LastState = State(x, y, heading, speed, t);
}

void Executive::planLoop() {
    double trialStartTime = m_TrajectoryPublisher->getTime(), cumulativeCollisionPenalty = 0;

    // Forget all dynamic obstacles. In practice this is not a good idea but for testing it's sort of OK
    // Roland, you probably want to take this out at some point and find a better way to make the planner forget about
    // old dynamic obstacles (timeout of some kind?)
    m_BinaryDynamicObstaclesManager = std::make_shared<BinaryDynamicObstaclesManager>();
    m_GaussianDynamicObstaclesManager = std::make_shared<GaussianDynamicObstaclesManager>();

    try {
        cerr << "Initializing planner" << endl;

        { // new scope to use RAII and not mess with later "lock" variable
            unique_lock<mutex> lock(m_PlannerStateMutex);
            m_CancelCV.wait_for(lock, chrono::seconds(2), [=] { return m_PlannerState != PlannerState::Cancelled; });
            if (m_PlannerState == PlannerState::Cancelled) {
                cerr << "Planner initialization timed out. Cancel flag is still set.\n" <<
                     "I think this happens when there was an error of some kind in the previous planning iteration.\n"
                     <<
                     "You're gonna have to restart the planner node if you want to keep using it.\n" << endl;
                return;
            }
            std::cerr << "Setting running state" << std::endl;
            m_PlannerState = PlannerState::Running;
        }

        State startState;
        // declare stats here so that the plan persists between loops
        Planner::Stats stats;

        // keep track of this so we can publish it with the stats
        bool lastPlanAchievable = false;

        // keep track of how many times in a row we fail to find a plan
        int failureCount = 0;

        while (true) {
            double startTime = m_TrajectoryPublisher->getTime();
            // logging time each time through the loop for making sure we're hitting the time bound
//            *m_PlannerConfig.output() << "Top of plan loop at time " << std::to_string(startTime) << std::endl;

            // planner is stateless so we can make a new instance each time
            unique_ptr<Planner> planner;
            if (m_UsePotentialField) {
                planner = std::unique_ptr<Planner>(new PotentialFieldPlanner);
            } else {
                planner = std::unique_ptr<Planner>(new AStarPlanner);
            }

            { // new scope for RAII again
                unique_lock<mutex> lock(m_PlannerStateMutex);
                if (m_PlannerState == PlannerState::Cancelled) {
                    // if we're not supposed to be running right now go ahead and break out
                    break;
                }
            }
            { // and again
                std::lock_guard<std::mutex> lock1(m_RibbonManagerMutex);
                if (m_RibbonManager.done()) {
                    // tell the node we're done
                    cerr << "Finished covering ribbons" << endl;
                    m_TrajectoryPublisher->allDone();
                    break;
                }
            }
            // display ribbons
            { // one more time
                std::lock_guard<std::mutex> lock1(m_RibbonManagerMutex);
                m_TrajectoryPublisher->displayRibbons(m_RibbonManager);
            }

            // if the state estimator returned an error naively do it ourselves
            if (startState.time() == -1) {
                startState = m_LastState.push(
                        m_TrajectoryPublisher->getTime() + c_PlanningTimeSeconds - m_LastState.time());
            }

            // copy the map pointer if it's been set (don't wait for the mutex because it may be a while)
            {
                std::unique_lock<std::mutex> lock1(m_MapMutex, std::defer_lock);
                if (lock1.try_lock()) {
                    if (m_NewMap) {
                        m_PlannerConfig.setMap(m_NewMap);
                    }
                    m_NewMap = nullptr;

                    // check if start state is blocked
                    // I don't remember why I put this inside the locked block but I'm sure I had a good reason
                    if (m_PlannerConfig.map()->isBlocked(startState.x(), startState.y())) {
                        *m_PlannerConfig.output() << "We've run aground, according to the most recent map!\n" <<
                        "Ending task now" << endl;
//                        cerr << "Starting state (" << startState.toString()
//                             << ") is blocked, according to most recent map. Trying again in 1s." << endl;
//                        sleep(1);
//                        continue;
                        m_TrajectoryPublisher->allDone();
                        break;
                    }
                }
            }

            if (!c_ReusePlanEnabled) stats.Plan = DubinsPlan();

            if (!stats.Plan.empty()) stats.Plan.changeIntoSuffix(startState.time()); // update the last plan

            // shrink turning radius (experimental)
            if (c_RadiusShrinkEnabled) {
                m_PlannerConfig.setTurningRadius(m_PlannerConfig.turningRadius() - c_RadiusShrinkAmount);
                m_PlannerConfig.setCoverageTurningRadius(
                        m_PlannerConfig.coverageTurningRadius() - c_RadiusShrinkAmount);
                m_RadiusShrink += c_RadiusShrinkAmount;
            }

            // check for collision penalty
            double collisionPenalty = 0;
            if (m_UseGaussianDynamicObstacles) {
                collisionPenalty = m_GaussianDynamicObstaclesManager->DynamicObstaclesManager::collisionExists(
                        m_LastState, false);
            } else {
                collisionPenalty = m_BinaryDynamicObstaclesManager->DynamicObstaclesManager::collisionExists(
                        m_LastState, false);
            }
            cumulativeCollisionPenalty += collisionPenalty;

            try {
                if (m_UseGaussianDynamicObstacles) {
                    m_PlannerConfig.setObstaclesManager(m_GaussianDynamicObstaclesManager);
                } else{
                    m_PlannerConfig.setObstaclesManager(m_BinaryDynamicObstaclesManager);
                }
                // display (binary) dynamic obstacles
//                for (auto o : m_BinaryDynamicObstaclesManager->get()) {
//                    auto& obstacle = o.second;
//                    obstacle.project(m_TrajectoryPublisher->getTime());
//                    m_TrajectoryPublisher->displayDynamicObstacle(obstacle.X, obstacle.Y, obstacle.Yaw, obstacle.Width, obstacle.Length, o.first);
//                }
                // TODO! -- display gaussian dynamic obstacles somehow

                // trying to fix seg fault by eliminating concurrent access to ribbon manager (seems to have fixed it)
                RibbonManager ribbonManagerCopy;
                {
                    std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
                    ribbonManagerCopy = m_RibbonManager;
                }
                // cover up to the state that we're planning from
                ribbonManagerCopy.coverBetween(m_LastState.x(), m_LastState.y(), startState.x(), startState.y(), false);
                stats = planner->plan(ribbonManagerCopy, startState, m_PlannerConfig, stats.Plan,
                                     startTime + c_PlanningTimeSeconds - m_TrajectoryPublisher->getTime());
            } catch (const std::exception& e) {
                cerr << "Exception thrown while planning:" << endl;
                cerr << e.what() << endl;
                cerr << "Ignoring that and just trying to proceed." << endl;
                stats.Plan = DubinsPlan();
            } catch (...) {
                cerr << "Unknown exception thrown while planning; pausing" << endl;
                cancelPlanner();
                throw;
            }

            m_TrajectoryPublisher->publishStats(stats, collisionPenalty * Edge::collisionPenaltyFactor(),
                                                0, lastPlanAchievable);

            // calculate remaining time (to sleep)
            double endTime = m_TrajectoryPublisher->getTime();
            int sleepTime = ((int) ((c_PlanningTimeSeconds - (endTime - startTime)) * 1000));
            if (sleepTime >= 0) {
//                *m_PlannerConfig.output() << "Finished with " << sleepTime << "ms extra time. Sleeping." << endl;
                this_thread::sleep_for(chrono::milliseconds(sleepTime));
            }
//            else {
//                *m_PlannerConfig.output() << "Failed to meet real-time bound by " << -sleepTime << "ms" << endl;
//            }

            // display the trajectory
            m_TrajectoryPublisher->displayTrajectory(stats.Plan.getHalfSecondSamples(), true, stats.Plan.dangerous());

            if (!stats.Plan.empty()) {
                failureCount = 0;
                // send trajectory to controller
                try {
                    startState = m_TrajectoryPublisher->publishPlan(stats.Plan);
                } catch (const std::exception& e) {
                    cerr << "Exception thrown while updating controller's reference trajectory:" << endl;
                    cerr << e.what() << endl;
                    cerr << "Pausing." << endl;
                    cancelPlanner();
                } catch (...) {
                    cerr << "Unknown exception thrown while updating controller's reference trajectory; pausing"
                         << endl;
                    cancelPlanner();
                    throw;
                }
                // if we cancelled the planner, the controller might not give us a valid next plan start, so we
                // should nope out now rather than fail with an exception in a couple of lines
                if (!stats.Plan.containsTime(startState.time())) {
                    unique_lock<mutex> lock2(m_PlannerStateMutex);
                    if (m_PlannerState == PlannerState::Cancelled) {
                        break;
                    }
                }
                State expectedStartState(startState);
                stats.Plan.sample(expectedStartState);
                if (!startState.isCoLocated(expectedStartState)) {
                    // reset plan because controller says we can't make it
                    stats.Plan = DubinsPlan();
                    lastPlanAchievable = false;

                    // reset turning radius shrink because we can't follow original plan anymore
                    if (c_RadiusShrinkEnabled) {
                        m_PlannerConfig.setTurningRadius(m_PlannerConfig.turningRadius() + m_RadiusShrink);
                        m_PlannerConfig.setCoverageTurningRadius(
                                m_PlannerConfig.coverageTurningRadius() + m_RadiusShrink);
                        m_RadiusShrink = 0;
                    }

                } else {
                    // expected start state is along plan so allow plan to be passed to planner as previous plan
                    m_RadiusShrink += c_RadiusShrinkAmount;
                    lastPlanAchievable = true;
                }
            } else {
                cerr << "Planner returned empty trajectory." << endl;
                startState = State();
                failureCount++;
                if (failureCount > 2) {
                    m_PlannerConfig.setTimeHorizon(m_PlannerConfig.timeHorizon() / 2);
                    if (m_PlannerConfig.timeHorizon() < m_PlannerConfig.timeMinimum())
                        // prevent from getting too small
                        m_PlannerConfig.setTimeHorizon(m_PlannerConfig.timeMinimum());
                    else {
                        cerr << "Failed " << failureCount << " times in a row. Reducing time horizon to "
                             << m_PlannerConfig.timeHorizon() << std::endl;
                        failureCount = 0;
                    }
                }
            }
        }
    }
    catch(const std::exception& e) {
        cerr << "Exception thrown in plan loop:" << endl;
        cerr << e.what() << endl;
        cerr << "Pausing." << endl;
        cancelPlanner();
    } catch (...) {
        cerr << "Unknown exception thrown in plan loop" << endl;
    }

    // task-level stats reporting
    auto trialEndTime = m_TrajectoryPublisher->getTime();
    auto wallClockTime = trialEndTime - trialStartTime;
    // multiply penalties by weights
    cumulativeCollisionPenalty *= Edge::collisionPenaltyFactor();
    auto timePenalty = wallClockTime * Edge::timePenaltyFactor();
    auto uncoveredLength = m_RibbonManager.getTotalUncoveredLength();

    m_TrajectoryPublisher->publishTaskLevelStats(wallClockTime, cumulativeCollisionPenalty,
                                                 timePenalty + cumulativeCollisionPenalty,
                                                 uncoveredLength);
    unique_lock<mutex> lock2(m_PlannerStateMutex);
    std::cerr << "Setting inactive state" << std::endl;
    m_PlannerState = PlannerState::Inactive;
    m_CancelCV.notify_all(); // do I need this?
}

void Executive::terminate()
{
    // cancel planner so thread can finish
    cancelPlanner();
}

void Executive::updateDynamicObstacle(uint32_t mmsi, State obstacle, double width, double length) {
//    m_DynamicObstaclesManager.update(mmsi, inventDistributions(obstacle));
    m_BinaryDynamicObstaclesManager->update(mmsi, obstacle.x(), obstacle.y(), obstacle.heading(),
            obstacle.speed(), obstacle.time(), width, length);
    m_GaussianDynamicObstaclesManager->update(mmsi, obstacle.x(), obstacle.y(), obstacle.heading(),
            obstacle.speed(), obstacle.time());
}

void Executive::refreshMap(const std::string& pathToMapFile, double latitude, double longitude) {
    // Run asynchronously and headless. The ol' fire-off-and-pray method
    thread([this, pathToMapFile, latitude, longitude] {
        std::lock_guard<std::mutex> lock(m_MapMutex);
        // skip re-loading if we've already loaded it
//        if (m_CurrentMapPath != pathToMapFile) {
            if (pathToMapFile.empty()) {
                m_NewMap = make_shared<Map>();
                m_CurrentMapPath = pathToMapFile;
                *m_PlannerConfig.output() << "Map cleared. Using empty map now." << endl;
                m_TrajectoryPublisher->displayMap(pathToMapFile);
                return;
            }
            // could take some time for I/O
            try {
                // If the name looks like it's one of our gridworld maps, load it in that format, otherwise assume GeoTIFF
                if ( access( pathToMapFile.c_str(), F_OK ) == -1 ) {
                    *m_PlannerConfig.output() << "Cannot find map file: " << pathToMapFile << endl;
                    *m_PlannerConfig.output() << "Using empty map  for now." << endl;
                    m_NewMap = make_shared<Map>();
                    m_CurrentMapPath = "";
                    m_TrajectoryPublisher->displayMap("");
                    return;
                }
                if (pathToMapFile.find(".map") == -1) {
                    // don't try to display geotiff maps
                    m_TrajectoryPublisher->displayMap("");
                    m_NewMap = make_shared<GeoTiffMap>(pathToMapFile, longitude, latitude);
                } else {
                    m_NewMap = make_shared<GridWorldMap>(pathToMapFile);
                    m_TrajectoryPublisher->displayMap(pathToMapFile);
                }
                m_CurrentMapPath = pathToMapFile;
                *m_PlannerConfig.output() << "Loaded map file: " << pathToMapFile << endl;
            }
            catch (...) {
                // swallow all errors in this thread
                *m_PlannerConfig.output() << "Encountered an error loading map at path " << pathToMapFile << ".\nMap was not updated." << endl;
                *m_PlannerConfig.output() << "Set the map path to an empty string to clear the map." << endl;
                m_NewMap = nullptr;
                m_CurrentMapPath = "";
            }
//        } else {
            // refresh display anyway
//            if (pathToMapFile.find(".map") != -1)
//                m_TrajectoryPublisher->displayMap(pathToMapFile);
//        }
    }).detach();
}

void Executive::addRibbon(double x1, double y1, double x2, double y2) {
    std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
    m_RibbonManager.add(x1, y1, x2, y2);
}

//std::vector<Distribution> Executive::inventDistributions(State obstacle) {
//    // This definitely needs some work. Maybe Distribution does too.
//    std::vector<Distribution> distributions;
//    double mean[2] = {obstacle.x(), obstacle.y()};
//    double covariance[2][2] = {{1, 0},{0, 1}};
//    distributions.emplace_back(mean, covariance, 5, 5, obstacle.heading(), obstacle.time());
//    obstacle = obstacle.push(1);
//    mean[0] = obstacle.x(); mean[1] = obstacle.y();
////    double covariance2[2][2] = {{2, 0}, {0, 2}}; // grow variance over time
//    distributions.emplace_back(mean, covariance, 5, 5, obstacle.heading(), obstacle.time());
//    return distributions;
//}

void Executive::clearRibbons() {
    std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
    m_RibbonManager = RibbonManager(RibbonManager::Heuristic::TspPointRobotNoSplitKRibbons, m_PlannerConfig.turningRadius(), 2);
}

void Executive::setConfiguration(double turningRadius, double coverageTurningRadius, double maxSpeed, double slowSpeed,
                                 double lineWidth, int k, int heuristic, double timeHorizon, double timeMinimum,
                                 double collisionCheckingIncrement, int initialSamples, bool useBrownPaths,
                                 bool useGaussianDynamicObstacles, bool ignoreDynamicObstacles,
                                 bool usePotentialField) {
    m_PlannerConfig.setTurningRadius(turningRadius);
    m_PlannerConfig.setCoverageTurningRadius(coverageTurningRadius);
    m_PlannerConfig.setMaxSpeed(maxSpeed);
    m_PlannerConfig.setSlowSpeed(slowSpeed);
    RibbonManager::setRibbonWidth(lineWidth);
    m_PlannerConfig.setBranchingFactor(k);
    switch (heuristic) {
        // check the .cfg file if this is breaking or if you change these
        case 0: m_RibbonManager.setHeuristic(RibbonManager::Heuristic::TspPointRobotNoSplitAllRibbons); break;
        case 1: m_RibbonManager.setHeuristic(RibbonManager::Heuristic::TspPointRobotNoSplitKRibbons); break;
        case 2: m_RibbonManager.setHeuristic(RibbonManager::Heuristic::MaxDistance); break;
        case 3: m_RibbonManager.setHeuristic(RibbonManager::Heuristic::TspDubinsNoSplitAllRibbons); break;
        case 4: m_RibbonManager.setHeuristic(RibbonManager::Heuristic::TspDubinsNoSplitKRibbons); break;
        default: *m_PlannerConfig.output() << "Unknown heuristic. Ignoring." << endl; break;
    }
    m_PlannerConfig.setTimeHorizon(timeHorizon);
    m_PlannerConfig.setTimeMinimum(timeMinimum);
    m_PlannerConfig.setCollisionCheckingIncrement(collisionCheckingIncrement);
    m_PlannerConfig.setInitialSamples(initialSamples);
    m_PlannerConfig.setUseBrownPaths(useBrownPaths);
    m_UseGaussianDynamicObstacles = useGaussianDynamicObstacles;
    m_IgnoreDynamicObstacles = ignoreDynamicObstacles;
    m_UsePotentialField = usePotentialField;
}

void Executive::startPlanner() {
    if (!m_PlannerConfig.map()) {
        m_PlannerConfig.setMap(make_shared<Map>());
    }
    std::unique_lock<mutex> lock(m_PlannerStateMutex);
    // only start a new thread if one isn't active
    // A thread could still be running but in the cancellation phase, which should be handled by the logic in planLoop
    if (m_PlannerState != PlannerState::Running)
        m_PlanningFuture = async(launch::async, &Executive::planLoop, this);
}

void Executive::cancelPlanner() {
    std::unique_lock<mutex> lock(m_PlannerStateMutex);
    if (m_PlannerState == PlannerState::Running) {
        m_PlannerState = PlannerState::Cancelled;
        std::cerr << "Setting cancelled state" << std::endl;
    }
}

void Executive::setPlannerVisualization(bool visualize, const std::string& visualizationFilePath) {
    m_PlannerConfig.setVisualizations(visualize);
    if (visualize) {
        m_Visualizer = Visualizer::UniquePtr(new Visualizer(visualizationFilePath));
        m_PlannerConfig.setVisualizer(&m_Visualizer);
    }
}

//void Executive::updateDynamicObstacle(uint32_t mmsi, const std::vector<Distribution>& obstacle) {
//    m_DynamicObstaclesManager.update(mmsi, obstacle);
//}

