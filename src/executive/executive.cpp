#include <thread>
#include <fstream>
#include <wait.h>
#include <future>
#include <rosconsole/macros_generated.h>
#include <memory>
#include "ExecutiveInternalsManager.h"
#include "executive.h"
#include "../planner/SamplingBasedPlanner.h"
#include "../planner/AStarPlanner.h"
#include "../common/map/GeoTiffMap.h"
#include "../common/map/GridWorldMap.h"

using namespace std;

Executive::Executive(TrajectoryPublisher *controlReceiver)
{
    m_TrajectoryPublisher = controlReceiver;

    m_PlannerConfig.setNowFunction([&] { return m_TrajectoryPublisher->getTime(); });

//    startThreads();
}

Executive::~Executive() {
    terminate();
    m_PlanningFuture.wait_for(chrono::seconds(2));
//    m_TrajectoryPublishingFuture.wait_for(chrono::seconds(1));
}

double Executive::getCurrentTime()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
//    m_InternalsManager.updateCurrent(x, y, speed, heading, t);
//    m_InternalsManager.updateCovered();
    if ((m_LastHeading - heading) / m_LastUpdateTime <= c_CoverageHeadingRateMax) {
        m_RibbonManager.cover(x, y);
    }
    m_LastUpdateTime = t; m_LastHeading = heading;
    m_LastState = State(x, y, heading, speed, t);
}

void Executive::planLoop() {
    cerr << "Initializing planner" << endl;

    auto planner = std::unique_ptr<Planner>(new AStarPlanner);

    { // new scope to use RAII and not mess with later "lock" variable
        unique_lock<mutex> lock(m_CancelLock);
        m_CancelCV.wait_for(lock, chrono::seconds(2), [=] { return !m_PlannerCancelled; });
        if (m_PlannerCancelled) {
            cerr << "Planner initialization timed out. Cancel flag is still set.\n" <<
                "This is likely the result of a race condition I haven't gotten around to fixing yet.\n" <<
                "You're gonna have to restart the planner node if you want to keep using it." << endl;
            return;
        }
    }

    cerr << "Starting plan loop" << endl;
    State startState(-1);

    while (true) {
        double startTime = m_TrajectoryPublisher->getTime();

        unique_lock<mutex> lock(m_CancelLock);
        if (m_PlannerCancelled) {
            m_PlannerCancelled = false;
            lock.unlock();
            break;
        }
        lock.unlock();

        if (m_RibbonManager.done()) {
            // tell the node we're done
            cerr << "Finished covering ribbons" << endl;
            m_TrajectoryPublisher->allDone();
            break;
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
        m_TrajectoryPublisher->displayRibbons(m_RibbonManager);

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

        // if the state estimator returns an error naively do it ourselves
        if (startState.time == -1) {
            startState.setEstimate(m_TrajectoryPublisher->getTime() + c_PlanningTimeSeconds - m_LastState.time, m_LastState);
        }

        try {
            // TODO! -- low-key data race with the ribbon manager here but it might be fine
            // its estimates of our trajectory
            m_PlannerConfig.setObstacles(m_DynamicObstaclesManager);
            plan = planner->plan(m_RibbonManager, startState, m_PlannerConfig,
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

        if (!plan.empty()) {
            cerr << "Sending trajectory to controller" << endl;
            startState = m_TrajectoryPublisher->publishTrajectory(plan);
            cerr << "Received state from controller: " << startState.toString() << endl;
        } else {
            cerr << "Planner returned empty trajectory." << endl;
            startState = State(-1);
        }

        // display the trajectory
        m_TrajectoryPublisher->displayTrajectory(plan, true);

        // calculate remaining time (to sleep)
        double endTime = m_TrajectoryPublisher->getTime();
        int sleepTime = (endTime - startTime <= c_PlanningTimeSeconds) ? ((int)((c_PlanningTimeSeconds - (endTime - startTime)) * 1000)) : 0;

        this_thread::sleep_for(chrono::milliseconds(sleepTime));
    }
}

// fix the moving of start
void Executive::requestPath()
{
    double start, end;
    int sleeptime;

//    m_InternalsManager.initialize();

    while (m_Running)
    {
        // if m_Pause, block until !m_Pause
        unique_lock<mutex> lk(m_PauseMutex);
        m_PauseCv.wait(lk, [=]{return !m_Pause;});
//        cerr << "requestPath unblocked (with the lock)" << endl;
        lk.unlock();
//        cerr << "requestPath released the lock" << endl;

        if (m_RibbonManager.done())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            cerr << "Finished path; pausing" << endl;
            pause();
            continue;
        }

        m_TrajectoryPublisher->displayRibbons(m_RibbonManager);

        if (m_MapMutex.try_lock()) {
            if (m_NewMap) {
                m_PlannerConfig.setMap(m_NewMap);
            }
            m_NewMap = nullptr;
            m_MapMutex.unlock();
        }

        start = m_TrajectoryPublisher->getTime();
        vector<State> plan;
//        cerr << "ROS time is now " << m_TrajectoryPublisher->getTime() << endl;
        auto startState = m_TrajectoryPublisher->getEstimatedState(m_TrajectoryPublisher->getTime() + c_PlanningTimeSeconds);
        if (startState.time == -1) { // if the state estimator returns an error naively do it ourselves
            startState.setEstimate(m_TrajectoryPublisher->getTime() + c_PlanningTimeSeconds - m_LastState.time, m_LastState);
        }
//        cerr << "Calling planner with state " << startState.toString() << endl;
        try {
            // TODO! -- low-key race condition with the ribbon manager here but it might be fine
            // NOTE: changed the time remaining from 0.95 to 0.7 to hopefully allow the controller to update
            // its estimates of our trajectory
            m_PlannerConfig.setObstacles(m_DynamicObstaclesManager);
            plan = m_Planner->plan(m_RibbonManager, startState, m_PlannerConfig,
                                   start + c_PlanningTimeSeconds - m_TrajectoryPublisher->getTime());
        }
        catch(const std::exception& e) {
            cerr << "Exception thrown while planning:" << endl;
            cerr << e.what() << endl;
            cerr << "Pausing." << endl;
            pause();
        }
        catch (...) {
            cerr << "Unknown exception thrown while planning; pausing" << endl;
            pause();
            throw;
        }

//        cerr << "Setting new path of length " << plan.size() << endl;
        if (!plan.empty()) {
            m_TrajectoryPublisher->publishTrajectory(plan);
        }
        m_TrajectoryPublisher->displayTrajectory(plan, true);
        end = m_TrajectoryPublisher->getTime();
        sleeptime = (end - start <= c_PlanningTimeSeconds) ? ((int)((c_PlanningTimeSeconds - (end - start)) * 1000)) : 0;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));
    }
    cerr << "Planner thread exiting." << endl;
}

void Executive::addToCover(int x, int y)
{
    m_InternalsManager.addToCover(x, y);
}

void Executive::startPlanner(const string& mapFile, double latitude, double longitude)
{
    cerr << "Starting planner" << endl;

    cerr << m_RibbonManager.dumpRibbons() << endl;

    shared_ptr<Map> map;
    try {
        map = make_shared<GeoTiffMap>(mapFile, longitude, latitude);
    }
    catch (...) {
        map = make_shared<Map>();
    }
    this->m_PlannerConfig.setMap(map);

    m_Planner = std::unique_ptr<Planner>(new AStarPlanner);

    // assume you've already set up path to cover
//    vector<pair<double, double>> toCover;
//    for (point p : m_InternalsManager.getCovered())
//        toCover.emplace_back(p.x, p.y);
//    m_Planner->addToCover(toCover);

    cerr << "Planner is up and running" << endl;

    // planner is running so the listener and updater threads should too
    unPause();
}

void Executive::startThreads()
{
//    cerr << "Starting thread to publish to controller" << endl;
//    m_TrajectoryPublishingFuture = async(launch::async, &Executive::sendAction, this);
    cerr << "Starting thread to listen to planner" << endl;
    m_PlanningFuture = async(launch::async, &Executive::requestPath, this);
}

void Executive::terminate()
{
//    if (!m_Running) return;
//    m_Running = false;

    // cancel planner so thread can finish
    cancelPlanner();

    // un-pause so threads can terminate
//    unPause();
}

void Executive::pause()
{
    m_PauseMutex.lock();
    if (m_Pause) {
        m_PauseMutex.unlock();
        return;
    }
    cancelPlanner();
    m_Pause = true;
    m_PauseMutex.unlock();

//    if (!m_Running) return;

    // tell the node we achieved the goal
//    m_TrajectoryPublisher->allDone();
}


bool Executive::plannerIsRunning() {
    return m_Running;
}

void Executive::unPause() {
    m_PauseMutex.lock();
    m_Pause = false;
    m_PauseMutex.unlock();
    m_PauseCv.notify_all();
}

void Executive::updateDynamicObstacle(uint32_t mmsi, State obstacle) {
//    m_InternalsManager.updateDynamicObstacle(mmsi, obstacle);
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
    m_RibbonManager.add(x1, y1, x2, y2);
}

std::vector<Distribution> Executive::inventDistributions(State obstacle) {
    std::vector<Distribution> distributions;
    double mean[2] = {obstacle.x, obstacle.y};
    double covariance[2][2] = {{0, 5},{5, 0}};
    distributions.emplace_back(mean, covariance, obstacle.heading, obstacle.time);
    obstacle.setEstimate(1, obstacle);
    mean[0] = obstacle.x; mean[1] = obstacle.y;
    distributions.emplace_back(mean, covariance, obstacle.heading, obstacle.time);
    return distributions;
}

void Executive::clearRibbons() {
    m_RibbonManager = RibbonManager();
}

void Executive::setVehicleConfiguration(double maxSpeed, double turningRadius, double coverageMaxSpeed,
                                        double coverageTurningRadius, int k) {
    m_PlannerConfig.setMaxSpeed(maxSpeed);
    m_PlannerConfig.setTurningRadius(turningRadius);
//    m_PlannerConfig.setCoverageMaxSpeed(coverageMaxSpeed); // deprecated
    m_PlannerConfig.setCoverageTurningRadius(coverageTurningRadius);
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
    std::unique_lock<mutex> lock(m_CancelLock);
    m_PlannerCancelled = true;
}

void Executive::setPlannerVisualization(bool visualize, const std::string& visualizationFilePath) {
    m_PlannerConfig.setVisualizations(visualize);
    if (visualize) {
        m_Visualizer = Visualizer::UniquePtr(new Visualizer(visualizationFilePath));
        m_PlannerConfig.setVisualizer(&m_Visualizer);
    }
}
