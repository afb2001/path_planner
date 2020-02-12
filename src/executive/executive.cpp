#include <thread>
#include <fstream>
#include <wait.h>
#include <future>
#include <memory>
#include "executive.h"
#include "../planner/SamplingBasedPlanner.h"
#include "../planner/AStarPlanner.h"
#include "../common/map/GeoTiffMap.h"
#include "../common/map/GridWorldMap.h"

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
        m_RibbonManager.cover(x, y);
    }
    m_LastUpdateTime = t; m_LastHeading = heading;
    m_LastState = State(x, y, heading, speed, t);
}

void Executive::planLoop() {
    cerr << "Initializing planner" << endl;

    auto planner = std::unique_ptr<Planner>(new AStarPlanner);

    { // new scope to use RAII and not mess with later "lock" variable
        unique_lock<mutex> lock(m_PlannerStateMutex);
        m_CancelCV.wait_for(lock, chrono::seconds(2), [=] { return m_PlannerState != PlannerState::Cancelled; });
        if (m_PlannerState == PlannerState::Cancelled) {
            cerr << "Planner initialization timed out. Cancel flag is still set.\n" <<
                "This is likely the result of a race condition I thought I fixed.\n" <<
                "You're gonna have to restart the planner node if you want to keep using it.\n" << endl;
            return;
        }
        m_PlannerState = PlannerState::Running;
    }

//    cerr << "Starting plan loop" << endl;
    State startState;


    while (true) {
        double startTime = m_TrajectoryPublisher->getTime();

//        std::cerr << "Top of plan loop" << std::endl;

        unique_lock<mutex> lock(m_PlannerStateMutex);
        if (m_PlannerState == PlannerState::Cancelled) {
            lock.unlock();
            break;
        }
        lock.unlock();

//        std::cerr << "Checking ribbons" << std::endl;
        {
            std::lock_guard<std::mutex> lock1(m_RibbonManagerMutex);
            if (m_RibbonManager.done()) {
                // tell the node we're done
                cerr << "Finished covering ribbons" << endl;
                m_TrajectoryPublisher->allDone();
                break;
            }
        }


        // display ribbons
        // seg fault has occurred in this call and I have no idea how (trace below)
        /*
            #0  0x00007f3c16e12cd5 in Ribbon::startAsState (this=0x10)
                at /home/abrown/project11/catkin_ws/src/path_planner/src/planner/utilities/Ribbon.cpp:55
            #1  0x00005582929167bb in PathPlanner::displayRibbons (this=0x7ffd7db6dc90,
                ribbonManager=...)
                at /home/abrown/project11/catkin_ws/src/path_planner/src/path_planner_node.cpp:270
            #2  0x00007f3c17078483 in Executive::planLoop (this=0x558293696800)
                at /home/abrown/project11/catkin_ws/src/path_planner/src/executive/executive.cpp:86
         */
        {
            std::lock_guard<std::mutex> lock1(m_RibbonManagerMutex);
            m_TrajectoryPublisher->displayRibbons(m_RibbonManager);
        }

//        std::cerr << "Displayed ribbons" << std::endl;

        // copy the map pointer if it's been set (don't wait for the mutex because it may be a while)
        if (m_MapMutex.try_lock()) {
            if (m_NewMap) {
                m_PlannerConfig.setMap(m_NewMap);
            }
            m_NewMap = nullptr;
            m_MapMutex.unlock();
        }

        // declare here so that I can assign it in try block and reference it later
        vector<State> plan;

        // call the controller service to get the estimated state we'll be in after the planning time has elapsed
//        auto startState = m_TrajectoryPublisher->getEstimatedState(m_TrajectoryPublisher->getTime() + c_PlanningTimeSeconds);

//        std::cerr << "Checking start state" << std::endl;
        // if the state estimator returns an error naively do it ourselves
        if (startState.time() == -1) {
            startState = m_LastState.push(m_TrajectoryPublisher->getTime() + c_PlanningTimeSeconds - m_LastState.time());
        }

        try {
            // TODO! -- low-key data race with the ribbon manager here but it might be fine
            // its estimates of our trajectory
            m_PlannerConfig.setObstacles(m_DynamicObstaclesManager);
            // trying to fix seg fault by eliminating concurrent access to ribbon manager (idk what the real problem is)
            RibbonManager ribbonManagerCopy;
            {
                std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
                ribbonManagerCopy = m_RibbonManager;
            }
            plan = planner->plan(ribbonManagerCopy, startState, m_PlannerConfig,
                                   startTime + c_PlanningTimeSeconds - m_TrajectoryPublisher->getTime());
        } catch(const std::exception& e) {
            cerr << "Exception thrown while planning:" << endl;
            cerr << e.what() << endl;
            cerr << "Pausing." << endl;
            pause();
        } catch (...) {
            cerr << "Unknown exception thrown while planning; pausing" << endl;
            pause();
            throw;
        }

        // calculate remaining time (to sleep)
        double endTime = m_TrajectoryPublisher->getTime();
        int sleepTime = (endTime - startTime <= c_PlanningTimeSeconds) ? ((int)((c_PlanningTimeSeconds - (endTime - startTime)) * 1000)) : 0;

        this_thread::sleep_for(chrono::milliseconds(sleepTime));

        if (!plan.empty()) {
//            cerr << "Sending trajectory to controller" << endl;
            startState = m_TrajectoryPublisher->publishTrajectory(plan);
//            cerr << "Received state from controller: " << startState.toString() << endl;
        } else {
            cerr << "Planner returned empty trajectory." << endl;
            startState = State();
        }

        // display the trajectory
        m_TrajectoryPublisher->displayTrajectory(plan, true);
    }

    unique_lock<mutex> lock2(m_PlannerStateMutex);
    m_PlannerState = PlannerState::Inactive;
    m_CancelCV.notify_all(); // do I need this?
}

void Executive::terminate()
{
    // cancel planner so thread can finish
    cancelPlanner();
}

void Executive::pause()
{
    cancelPlanner();
}

void Executive::unPause() {
    m_PauseMutex.lock();
    m_Pause = false;
    m_PauseMutex.unlock();
//    m_PauseCv.notify_all();
}

void Executive::updateDynamicObstacle(uint32_t mmsi, State obstacle) {
    m_DynamicObstaclesManager.update(mmsi, inventDistributions(obstacle));
}

void Executive::refreshMap(std::string pathToMapFile, double latitude, double longitude) {
    thread([this, pathToMapFile, latitude, longitude] {
        std::lock_guard<std::mutex> lock(m_MapMutex);
        if (m_CurrentMapPath != pathToMapFile) {
            // could take some time for I/O, Dijkstra on entire map
            try {
                // If the name looks like it's one of our gridworld maps, load it in that format, otherwise assume GeoTIFF
                if (pathToMapFile.find(".map") == -1) {
                    m_NewMap = make_shared<GeoTiffMap>(pathToMapFile, longitude, latitude);
                } else {
                    m_NewMap = make_shared<GridWorldMap>(pathToMapFile);
                }
                m_CurrentMapPath = pathToMapFile;
            }
            catch (...) {
                // swallow all errors in this thread
                cerr << "Encountered an error loading map at path " << pathToMapFile << endl;
                m_NewMap = nullptr;
                m_CurrentMapPath = "";
            }
        }
    }).detach();
}

void Executive::addRibbon(double x1, double y1, double x2, double y2) {
    std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
    m_RibbonManager.add(x1, y1, x2, y2);
}

std::vector<Distribution> Executive::inventDistributions(State obstacle) {
    std::vector<Distribution> distributions;
    double mean[2] = {obstacle.x(), obstacle.y()};
    double covariance[2][2] = {{0, 5},{5, 0}};
    distributions.emplace_back(mean, covariance, obstacle.heading(), obstacle.time());
    obstacle = obstacle.push(1);
    mean[0] = obstacle.x(); mean[1] = obstacle.y();
    distributions.emplace_back(mean, covariance, obstacle.heading(), obstacle.time());
    return distributions;
}

void Executive::clearRibbons() {
    std::lock_guard<std::mutex> lock(m_RibbonManagerMutex);
    m_RibbonManager = RibbonManager(RibbonManager::Heuristic::TspPointRobotNoSplitKRibbons, m_PlannerConfig.turningRadius(), 2);
}

void Executive::setVehicleConfiguration(double turningRadius, double coverageTurningRadius, double maxSpeed,
        double lineWidth, int k) {
    m_PlannerConfig.setMaxSpeed(maxSpeed);
    m_PlannerConfig.setTurningRadius(turningRadius);
    m_PlannerConfig.setCoverageTurningRadius(coverageTurningRadius);
    RibbonManager::setRibbonWidth(lineWidth);
    m_PlannerConfig.setBranchingFactor(k);
}

void Executive::startPlanner() {
    unPause();
    if (!m_PlannerConfig.map()) {
        m_PlannerConfig.setMap(make_shared<Map>());
    }
    m_PlanningFuture = async(launch::async, &Executive::planLoop, this);
}

void Executive::cancelPlanner() {
    std::unique_lock<mutex> lock(m_PlannerStateMutex);
    if (m_PlannerState == PlannerState::Running)
        m_PlannerState = PlannerState::Cancelled;
}

void Executive::setPlannerVisualization(bool visualize, const std::string& visualizationFilePath) {
    m_PlannerConfig.setVisualizations(visualize);
    if (visualize) {
        m_Visualizer = Visualizer::UniquePtr(new Visualizer(visualizationFilePath));
        m_PlannerConfig.setVisualizer(&m_Visualizer);
    }
}
