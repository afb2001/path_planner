#include <utility>
#include <thread>
#include <string.h>
//#include "State.h"
#include "communication.h"
#include <fstream>
#include <wait.h>
#include <pwd.h>
#include "path.h"
// #include "xtiffio.h"
// #include "geotiffio.h"

#include "executive.h"

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

void Executive::print_map(string file)
{
    if (file != "NOFILE")
    {
        string line;
        ifstream f(file);
        if (f.is_open())
        {
            getline(f, line);
            string factor = line;
            getline(f, line);
            string w = line;
            getline(f, line);
            string h = line;
            cerr << "EXEUTIVE::START " << w << " " << h << endl;
            cerr << "EXECUTIVE::MAP::" + w + " " + h << endl;
            int width = stoi(w), height = stoi(h);
            m_Path.maxX = width;
            m_Path.Obstacles = new bool[width * height];
            int hcount = 0;
            m_PipeToPlanner.cwrite("map " + factor + " " + w + " " + h);
            while (getline(f, line))
            {
                ++hcount;
                string s = "";
                char previous = ' ';
                int ncount = 0;
                for (int i = 0; i < line.size(); i++)
                {
                    m_Path.Obstacles[m_Path.getindex(i, height - hcount)] = line[i] == '#';

                    if (line[i] != previous)
                    {
                        if (i == 0)
                            s += line[i];
                        else
                            s += " " + to_string(ncount);
                        previous = line[i];
                    }
                    ncount += 1;
                }
                m_PipeToPlanner.cwrite(s);
            }

            f.close();
            return;
        }
    }
    string s = "";
    cerr << "EXECUTIVE::MAP::DEFAULT" << endl;
    m_PipeToPlanner.cwrite("map 1 2000 2000");
    for (int i = 0; i < 1999; i++)
        s += "_\n";
    s += "_";
    m_Path.Obstacles = new bool[2000 * 2000]{};
    m_PipeToPlanner.cwrite(s);
}

bool Executive::plannerIsDead()
{
    int status;
    pid_t result = waitpid(m_PipeToPlanner.getPid(), &status, WNOHANG);
//    if (result != 0) {
//        cerr << "Planner seems to be dead" << endl;
//    } else {
//        cerr << "Planner seems to be alive" << endl;
//    }
    return result != 0; // TODO! -- check error case
}

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
    m_Path.update_current(x,y,speed,heading,t);
    m_Path.update_covered();
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
        auto actions = m_Path.getActions();
//        cerr << "sendAction got path actions" << endl;
        if (actions.size() == 1)
            continue;
//        cerr << "and they aren't null" << endl;
        m_TrajectoryPublisher->publishTrajectory(actions);
        m_TrajectoryPublisher->displayTrajectory(actions);
//        cerr << "trajectory had length " << actions.size() << endl;
        this_thread::sleep_for(std::chrono::milliseconds(sleep));

//        cerr << "sendAction asking if planner is dead" << endl;
        if (plannerIsDead()) pause();
    }
}

// fix the moving of start
void Executive::requestPath()
{
    double start, end, time_bound;
    int numberOfState, sleeptime;
    char response[1024];

    m_Path.initialize();

    while (m_Running)
    {
        // if m_Pause, block until !m_Pause
        unique_lock<mutex> lk(m_PauseMutex);
        m_PauseCV.wait(lk, [=]{return !m_Pause;});
//        cerr << "requestPath unblocked (with the lock)" << endl;
        lk.unlock();
//        cerr << "requestPath released the lock" << endl;

        if (m_Path.finish())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            cerr << "Finished path; pausing" << endl;
            pause();
        }

        start = getCurrentTime();
        m_PipeToPlanner.cwrite(m_Path.construct_request_string());
//        cerr << "requestPath sent the planner a request" << endl;

//        fgets(response, sizeof response, readstream);
        m_PipeToPlanner.readAll(response);
//        cerr << "requestPath read the response" << endl;
        if (!strncmp(response, "done", 4))
        {
            cerr << "Unexpected answer from planner; pausing" << endl;
            pause();
        }

        sscanf(response, "plan %d\n", &numberOfState);
//        cerr << "which was a plan of length " << numberOfState << endl;

        time_bound = m_Path.getCurrent().time;

        vector<State> trajectory;
        for (int i = 0; i < numberOfState; i++) // if no new path then keep old path // ??
        {
//            fgets(response, sizeof response, readstream);
            m_PipeToPlanner.readAll(response);
            auto s = Path::readStateFromPlanner(response);
            if (s.time > time_bound) trajectory.push_back(s);
        }

        m_Path.setNewPath(trajectory);
//        m_TrajectoryPublisher->displayTrajectory(trajectory);

        end = getCurrentTime();
        sleeptime = (numberOfState) ? ((end - start <= 1) ? ((int)((1 - (end - start)) * 1000)) : 0) : 50;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));

//        cerr << "requestPath asking if planner is dead" << endl;
        if (plannerIsDead()) pause();
    }
}

void Executive::addToCover(int x, int y)
{
    m_Path.add_covered(x, y);
}

void Executive::startPlanner(string mapFile)
{
    cerr << "Starting planner" << endl;

    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    m_PipeToPlanner.set(string(homedir) + "/go/src/github.com/afb2001/CCOM_planner/planner", true, true, false, false);
    m_PipeToPlanner.cwrite("Start");
    m_PipeToPlanner.cwrite("max speed 2.3");
    m_PipeToPlanner.cwrite("max turning radius 8");
    print_map(std::move(mapFile));

    // assume you've already set up path to cover
    m_PipeToPlanner.cwrite("path to cover " + to_string(m_Path.getToCover().size()));
    for (point p : m_Path.getToCover())
        m_PipeToPlanner.cwrite(to_string(p.x) + " " + to_string(p.y));

    cerr << "Waiting for planner to finish starting" << endl;

    // wait for it to finish initializing (?)
    char done[100];
    m_PipeToPlanner.cread(done, 100);
    m_PlannerPipeStale = true;

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

    // if the planner isn't dead, kill it
    int status;
    pid_t result = waitpid(m_PipeToPlanner.getPid(), &status, WNOHANG);
    if (result == 0) {
        kill(m_PipeToPlanner.getPid(), SIGKILL);
    }
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

    // if the planner isn't dead, kill it
    int status;
    pid_t result = waitpid(m_PipeToPlanner.getPid(), &status, WNOHANG);
    if (result == 0) {
        kill(m_PipeToPlanner.getPid(), SIGKILL);
    }
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
    m_Path.updateDynamicObstacle(mmsi, obstacle);
}
