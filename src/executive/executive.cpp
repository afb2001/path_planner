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

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
    request_start = true;
    m_InternalsManager.update_current(x,y,speed,heading,t);
    m_InternalsManager.update_covered();
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
        auto actions = m_InternalsManager.getActions();
//        cerr << "sendAction got path actions" << endl;
        if (actions.empty())
            continue;
//        cerr << "and they aren't null" << endl;
        m_TrajectoryPublisher->publishTrajectory(actions);
//        cerr << "sendAction published trajectory" << endl;
        this_thread::sleep_for(std::chrono::milliseconds(sleep));

//        cerr << "sendAction asking if planner is dead" << endl;
//        if (plannerIsDead()) pause();
    }
}

// fix the moving of start
void Executive::requestPath()
{
    double start, end;
    int sleeptime;

    m_InternalsManager.initialize();

    while (m_Running)
    {
        // if m_Pause, block until !m_Pause
        unique_lock<mutex> lk(m_PauseMutex);
        m_PauseCV.wait(lk, [=]{return !m_Pause;});
//        cerr << "requestPath unblocked (with the lock)" << endl;
        lk.unlock();
//        cerr << "requestPath released the lock" << endl;

        if (m_InternalsManager.finish())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            cerr << "Finished path; pausing" << endl;
            pause();
        }

        start = getCurrentTime();
        auto newlyCoveredList = m_InternalsManager.getNewlyCovered();
        vector<pair<double, double>> newlyCovered;
        for (auto p : newlyCoveredList) newlyCovered.emplace_back(p.x, p.y);
        vector<State> plan;
        try {
            // change this line to use controller's starting estimate
            plan = m_Planner->plan(newlyCovered, m_InternalsManager.getStart(), DynamicObstaclesManager(), 0.95);
//            plan = m_Planner->plan(newlyCovered, m_TrajectoryPublisher->getEstimatedState(getCurrentTime() + 1), DynamicObstaclesManager());
        } catch (...) {
            cerr << "Exception thrown while planning; pausing" << endl;
            pause();
            throw;
        }

        cerr << "Setting new path of length " << plan.size() << endl;
        m_InternalsManager.setNewPath(plan);
//        m_TrajectoryPublisher->publishTrajectory(plan);
        m_TrajectoryPublisher->displayTrajectory(plan, true);
        end = getCurrentTime();
        sleeptime = (end - start <= 1) ? ((int)((1 - (end - start)) * 1000)) : 0;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));

//        if (plannerIsDead()) pause();
    }
}

void Executive::addToCover(int x, int y)
{
    m_InternalsManager.add_covered(x, y);
}

void Executive::startPlanner(string mapFile)
{
    cerr << "Starting planner" << endl;

//    m_Planner = std::unique_ptr<Planner>(new Planner(2.3, 8, Map()));
    m_Planner = std::unique_ptr<Planner>(new AStarPlanner(2.3, 8, Map()));

    // assume you've already set up path to cover
    vector<pair<double, double>> toCover;
    for (point p : m_InternalsManager.get_covered())
        toCover.emplace_back(p.x, p.y);
    m_Planner->addToCover(toCover);

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

void Executive::updateDyamicObstacle(uint32_t mmsi, State obstacle) {
    m_InternalsManager.updateDynamicObstacle(mmsi, obstacle);
}
