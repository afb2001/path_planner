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
            path.Maxx = width;
            path.Obstacles = new bool[width * height];
            int hcount = 0;
            communication_With_Planner.cwrite("map " + factor + " " + w + " " + h);
            while (getline(f, line))
            {
                ++hcount;
                string s = "";
                char previous = ' ';
                int ncount = 0;
                for (int i = 0; i < line.size(); i++)
                {
                    path.Obstacles[path.getindex(i, height - hcount)] = line[i] == '#';

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
                communication_With_Planner.cwrite(s);
            }

            f.close();
            return;
        }
    }
    string s = "";
    cerr << "EXECUTIVE::MAP::DEFAULT" << endl;
    communication_With_Planner.cwrite("map 1 2000 2000");
    for (int i = 0; i < 1999; i++)
        s += "_\n";
    s += "_";
    path.Obstacles = new bool[2000 * 2000]{};
    communication_With_Planner.cwrite(s);
}

bool Executive::plannerIsDead()
{
    int status;
    pid_t result = waitpid(communication_With_Planner.getPid(), &status, WNOHANG);
//    if (result != 0) {
//        cerr << "Planner seems to be dead" << endl;
//    } else {
//        cerr << "Planner seems to be alive" << endl;
//    }
    return result != 0; // TODO! -- check error case
}

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
    request_start = true;
    path.update_current(x,y,speed,heading,t);
    path.update_covered();
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
        if (actions.size() == 1)
            continue;
//        cerr << "and they aren't null" << endl;
        m_TrajectoryPublisher->publishTrajectory(actions);
//        cerr << "sendAction published trajectory" << endl;
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

    path.initialize();

    while (m_Running)
    {
        // if m_Pause, block until !m_Pause
        unique_lock<mutex> lk(m_PauseMutex);
        m_PauseCV.wait(lk, [=]{return !m_Pause;});
//        cerr << "requestPath unblocked (with the lock)" << endl;
        lk.unlock();
//        cerr << "requestPath released the lock" << endl;

        if (path.finish())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            cerr << "Finished path; pausing" << endl;
            pause();
        }

        start = getCurrentTime();
        communication_With_Planner.cwrite(path.construct_request_string());
//        cerr << "requestPath sent the planner a request" << endl;

//        fgets(response, sizeof response, readstream);
        communication_With_Planner.readAll(response);
//        cerr << "requestPath read the response" << endl;
        if (!strncmp(response, "done", 4))
        {
            cerr << "Unexpected answer from planner; pausing" << endl;
            pause();
        }

        sscanf(response, "plan %d\n", &numberOfState);
//        cerr << "which was a plan of length " << numberOfState << endl;

        time_bound = path.getCurrent().otime;

        cerr << "Updating reference trajectory for controller" << endl;
        vector<State> trajectory;
        for (int i = 0; i < numberOfState; i++) // if no new path then keep old path // ??
        {
//            fgets(response, sizeof response, readstream);
            communication_With_Planner.readAll(response);
            trajectory.push_back(Path::readStateFromPlanner(response, time_bound));
        }

        path.setNewPath(trajectory);
        m_TrajectoryPublisher->displayTrajectory(trajectory);

        end = getCurrentTime();
        sleeptime = (numberOfState) ? ((end - start <= 1) ? ((int)((1 - (end - start)) * 1000)) : 0) : 50;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));

//        cerr << "requestPath asking if planner is dead" << endl;
        if (plannerIsDead()) pause();
    }
}

void Executive::addToCover(int x, int y)
{
    path.add_covered(x, y);
}

void Executive::startPlanner(string mapFile)
{
    cerr << "Starting planner" << endl;

    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    communication_With_Planner.set(string(homedir) + "/go/src/github.com/afb2001/CCOM_planner/planner", true, true, false, false);
    communication_With_Planner.cwrite("Start");
    communication_With_Planner.cwrite("max speed 2.3");
    communication_With_Planner.cwrite("max turning radius 8");
    print_map(std::move(mapFile));

    // assume you've already set up path to cover
    communication_With_Planner.cwrite("path to cover " + to_string(path.get_covered().size()));
    for (point p : path.get_covered())
        communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));

    cerr << "Waiting for planner to finish starting" << endl;

    // wait for it to finish initializing (?)
    char done[100];
    communication_With_Planner.cread(done, 100);
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
    pid_t result = waitpid(communication_With_Planner.getPid(), &status, WNOHANG);
    if (result == 0) {
        kill(communication_With_Planner.getPid(), SIGKILL);
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
    pid_t result = waitpid(communication_With_Planner.getPid(), &status, WNOHANG);
    if (result == 0) {
        kill(communication_With_Planner.getPid(), SIGKILL);
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
    path.updateDynamicObstacle(mmsi, obstacle);
}
