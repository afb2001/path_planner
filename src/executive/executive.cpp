#include <utility>
#include <thread>
#include <string.h>
//#include "State.h"
//#include "communication.h"
#include <fstream>
#include <wait.h>
#include <pwd.h>
#include "path.h"
// #include "xtiffio.h"
// #include "geotiffio.h"

#include "executive.h"
#include "../planner/SamplingBasedPlanner.h"
#include "../planner/AStarPlanner.h"
#include "../common/map/GeoTiffMap.h"
#include "../common/map/GridWorldMap.h"

using namespace std;

Executive::Executive(TrajectoryPublisher *controlReceiver)
{
    m_TrajectoryPublisher = controlReceiver;

    startThreads();
}

Executive::~Executive() {
    terminate();
}

double Executive::getCurrentTime()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

//void Executive::print_map(string file)
//{
//    if (file != "NOFILE")
//    {
//        string line;
//        ifstream f(file);
//        if (f.is_open())
//        {
//            getline(f, line);
//            string factor = line;
//            getline(f, line);
//            string w = line;
//            getline(f, line);
//            string h = line;
//            cerr << "EXEUTIVE::START " << w << " " << h << endl;
//            cerr << "EXECUTIVE::MAP::" + w + " " + h << endl;
//            int width = stoi(w), height = stoi(h);
//            path.Maxx = width;
//            path.Obstacles = new bool[width * height];
//            int hcount = 0;
//            communication_With_Planner.cwrite("map " + factor + " " + w + " " + h);
//            while (getline(f, line))
//            {
//                ++hcount;
//                string s = "";
//                char previous = ' ';
//                int ncount = 0;
//                for (int i = 0; i < line.size(); i++)
//                {
//                    path.Obstacles[path.getindex(i, height - hcount)] = line[i] == '#';
//
//                    if (line[i] != previous)
//                    {
//                        if (i == 0)
//                            s += line[i];
//                        else
//                            s += " " + to_string(ncount);
//                        previous = line[i];
//                    }
//                    ncount += 1;
//                }
//                communication_With_Planner.cwrite(s);
//            }
//
//            f.close();
//            return;
//        }
//    }
//    string s = "";
//    cerr << "EXECUTIVE::MAP::DEFAULT" << endl;
//    communication_With_Planner.cwrite("map 1 2000 2000");
//    for (int i = 0; i < 1999; i++)
//        s += "_\n";
//    s += "_";
//    path.Obstacles = new bool[2000 * 2000]{};
//    communication_With_Planner.cwrite(s);
//}

//bool Executive::plannerIsDead()
//{
//    int status;
//    pid_t result = waitpid(communication_With_Planner.getPid(), &status, WNOHANG);
////    if (result != 0) {
////        cerr << "Planner seems to be dead" << endl;
////    } else {
////        cerr << "Planner seems to be alive" << endl;
////    }
//    return result != 0; // TODO! -- check error case
//}

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
    request_start = true;
    path.update_current(x,y,speed,heading,t);
    path.update_covered();
    if ((m_LastHeading - heading) / m_LastUpdateTime <= c_CoverageHeadingRateMax) {
        m_RibbonManager.cover(x, y);
    }
    m_LastUpdateTime = t; m_LastHeading = heading;
}

void Executive::sendAction() {
    int sleep = 50; // for 20 Hz
    while (m_Running)
    {
        // if m_Pause, block until !m_Pause
        unique_lock<mutex> lk(m_PauseMutex);
        m_PauseCV.wait(lk, [=]{return !m_Pause;});
//        cerr << "sendAction unblocked (with the lock)" << endl;
        lk.unlock();
//        cerr << "sendAction released the lock" << endl;
        auto actions = path.getActions();
//        cerr << "sendAction got path actions" << endl;
        if (actions == nullptr)
            continue;
//        cerr << "and they aren't null" << endl;
        vector<State> a;
        for (int i = 0; i < 5; i++) a.push_back(actions[i]);
        m_TrajectoryPublisher->publishTrajectory(a);
//        cerr << "sendAction published trajectory" << endl;
        this_thread::sleep_for(std::chrono::milliseconds(sleep));

//        cerr << "sendAction asking if planner is dead" << endl;
//        if (plannerIsDead()) pause();
        delete[] actions;
    }
}

// fix the moving of start
void Executive::requestPath()
{
    double start, end, time_bound;
    int numberOfState, sleeptime;
    char response[1024];

    path.initialize();

    while (m_Running)
    {
//        cerr << "Checking to make sure planner is not paused..." << endl;
        // if m_Pause, block until !m_Pause
        unique_lock<mutex> lk(m_PauseMutex);
        m_PauseCV.wait(lk, [=]{return !m_Pause;});
//        cerr << "requestPath unblocked (with the lock)" << endl;
        lk.unlock();
//        cerr << "requestPath released the lock" << endl;

        if (m_RibbonManager.done())
//        if (m_InternalsManager.finish())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            cerr << "Finished path; pausing" << endl;
            pause();
            continue;
        }

        if (m_MapMutex.try_lock()) {
            if (m_NewMap) {
                m_Planner->updateMap(m_NewMap);
            }
            m_NewMap = nullptr;
            m_MapMutex.unlock();
        }

        start = getCurrentTime();
        auto newlyCoveredList = path.getNewlyCovered();
        vector<pair<double, double>> newlyCovered;
        for (auto p : newlyCoveredList) newlyCovered.emplace_back(p.x, p.y);
        vector<State> plan;
        auto startState = path.getStart(); // m_TrajectoryPublisher->getEstimatedState(getCurrentTime() + 1);
        try {
            cerr << "Planning..." << endl;
            // change this line to use controller's starting estimate
            //            plan = m_Planner->plan(newlyCovered, m_TrajectoryPublisher->getEstimatedState(getCurrentTime() + 1), DynamicObstaclesManager());
//            plan = m_Planner->plan(newlyCovered, m_InternalsManager.getStart(), DynamicObstaclesManager(), 0.95);
            // TODO! -- low-key race condition with the ribbon manager here but it might be fine
            plan = m_Planner->plan(m_RibbonManager, startState, DynamicObstaclesManager(), 0.95);
        } catch (...) {
            cerr << "Exception thrown while planning; pausing" << endl;
            pause();
            throw;
        }
//        cerr << "Done planning" << endl;

        path.setNewPath(plan);
//        m_TrajectoryPublisher->publishTrajectory(plan);
        m_TrajectoryPublisher->displayTrajectory(plan, true);
//        cerr << "Plan: " << '\n';
//        for (auto s : plan) {
//            cerr << s.toString() << '\n';
//        }
//        cerr << endl;
        end = getCurrentTime();
        sleeptime = (end - start <= 1) ? ((int)((1 - (end - start)) * 1000)) : 0;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));

//        if (plannerIsDead()) pause();
    }
}

void Executive::addToCover(int x, int y)
{
    path.add_covered(x, y);
}

void Executive::startPlanner(const string& mapFile, double latitude, double longitude)
{
    cerr << "Starting planner" << endl;

    shared_ptr<Map> map;
    try {
        map = make_shared<GeoTiffMap>(mapFile, longitude, latitude);
    }
    catch (...) {
        map = make_shared<Map>();
    }
//    m_Planner = std::unique_ptr<Planner>(new Planner(2.3, 8, Map()));
    m_Planner = std::unique_ptr<Planner>(new AStarPlanner(2.3, 8, map));

    if (!m_Planner) {
        cerr << "Error creating planner! Planner not initialized!" << endl;
    }

    // assume you've already set up path to cover
//    vector<pair<double, double>> toCover;
//    for (point p : path.get_covered())
//        toCover.emplace_back(p.x, p.y);
//    m_Planner->addToCover(toCover);

    cerr << "Starting " << m_RibbonManager.dumpRibbons() << endl;

    cerr << "Planner is up and running" << endl;

    // planner is running so the listener and updater threads should too
    unPause();
}

void Executive::startThreads()
{
    m_Running = true;
    cerr << "Starting thread to publish to controller" << endl;
    thread thread_for_controller(thread([=] { sendAction(); }));
    thread_for_controller.detach();
    cerr << "Starting thread to listen to planner" << endl;
    thread thread_for_planner(thread([=] { requestPath(); }));
    thread_for_planner.detach();
}

void Executive::terminate()
{
    if (!m_Running) return;
    m_Running = false;

    // un-pause so threads can terminate
    unPause();
}

void Executive::pause()
{
    m_PauseMutex.lock();
    if (m_Pause) {
        m_PauseMutex.unlock();
        return;
    }
    m_Pause = true;
    m_PauseMutex.unlock();

    if (!m_Running) return;

    // tell the node we achieved the goal
    m_TrajectoryPublisher->allDone();
}


bool Executive::plannerIsRunning() {
    return m_Running;
}

void Executive::unPause() {
    m_PauseMutex.lock();
    m_Pause = false;
    m_PauseMutex.unlock();
    m_PauseCV.notify_all();
}

void Executive::updateDynamicObstacle(uint32_t mmsi, State obstacle) {
    m_DynamicObstaclesManager.update(mmsi, inventDistributions(obstacle));
}

void Executive::refreshMap(std::string pathToMapFile, double latitude, double longitude) {
    thread worker([this, pathToMapFile, latitude, longitude] {
        std::lock_guard<std::mutex> lock(m_MapMutex);
        // could take some time for I/O, Dijkstra on entire map
        try {
            // If the name looks like it's one of our gridworld maps, load it in that format, otherwise assume GeoTIFF
            if (pathToMapFile.find(".map") == -1) {
                m_NewMap = make_shared<GeoTiffMap>(pathToMapFile, longitude, latitude);
            } else {
                m_NewMap = make_shared<GridWorldMap>(pathToMapFile);
            }
        }
        catch (...) {
            // swallow all errors in this thread
            cerr << "Encountered an error loading map at path " << pathToMapFile << endl;
            m_NewMap = nullptr;
        }
    });
    worker.detach();
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
    m_RibbonManager = RibbonManager(RibbonManager::TspPointRobotNoSplitAllRibbons);
}
