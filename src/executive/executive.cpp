#include <utility>

#include <utility>

//#include <iostream>
//#include <limits>
//#include <chrono>
#include <thread>
//#include <mutex>
//#include <queue>
//#include <string>
//#include <sstream>
//#include <cmath>
//#include <ctime>
//#include <sys/time.h>
//#include <time.h>
#include <string.h>
//#include "ObjectPar.h"
#include "communication.h"
#include <fstream>
#include <wait.h>
//#include <list>
//#include <unordered_set>
#include "path.h"
// #include "xtiffio.h"
// #include "geotiffio.h"

#include "executive.h"

using namespace std;

Executive::Executive(ControlReceiver *controlReceiver)
{
    m_ControlReceiver = controlReceiver;
    m_Controller = new Controller(controlReceiver);
}

Executive::~Executive()
{
    delete m_Controller;
}

double Executive::getCurrentTime()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void Executive::checkTerminate()
{
    if (getppid() == 1)
    {
        cerr << "executive TERMINATE" << endl;
        running = false;
        exit(1);
    }
    int status;
    pid_t result = waitpid(communication_With_Planner.getPid(), &status, WNOHANG);
    if (result) { // TODO! -- check error case?
        finished();
    }
}

void Executive::updateCovered(double x, double y, double speed, double heading, double t)
{
    request_start = true;
    path.update_current(x,y,speed,heading,t);
    path.update_covered();
}

void Executive::sendAction()
{
    int sleep = 50; // for 20 Hz
    while (running)
    {
        auto actions = path.getActions();
        if (actions == nullptr)
            continue;
        m_Controller->receiveRequest(actions);
        this_thread::sleep_for(std::chrono::milliseconds(sleep));
        if (!pause_all) {
            checkTerminate();
        }
    }
}

// fix the moving of start
void Executive::requestPath()
{
    while (!request_start)
        this_thread::sleep_for(std::chrono::milliseconds(50));

    FILE *readstream = fdopen(communication_With_Planner.getWpipe(), "r");
    double start, end, time_bound;
    int numberOfState, sleeptime;
    char response[1024];

    path.initialize();

    while (running)
    {
        if(pause_all)
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            continue;
        }
        if (path.finish())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            finished();
        }

        start = getCurrentTime();
        communication_With_Planner.cwrite(path.construct_request_string());

        fgets(response, sizeof response, readstream);
        if (!strncmp(response, "done", 4))
        {
            finished();
        }

        sscanf(response, "plan %d\n", &numberOfState);

        time_bound = path.getCurrent().otime;

        cerr << "Updating reference trajectory for controller" << endl;
        for (int i = 0; i < numberOfState; i++) // if no new path then keep old path // ??
        {
            fgets(response, sizeof response, readstream);
            path.update_newpath(response, time_bound);
        }

        end = getCurrentTime();
        sleeptime = (numberOfState) ? ((end - start <= 1) ? ((int)((1 - (end - start)) * 1000)) : 0) : 50;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));
        checkTerminate();
    }
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

void Executive::addToCover(int x, int y)
{
    path.add_covered(x, y);
}

void Executive::startPlanner(string mapFile)
{
    running = true;

//    cerr << "Starting planner" << endl;
    // TODO! -- system-independent location of planner executable
    communication_With_Planner.set("/home/alex/go/src/github.com/afb2001/CCOM_planner/executive/planner", true, true, false, false);
    communication_With_Planner.cwrite("Start");
    communication_With_Planner.cwrite("max speed 2.3");
    communication_With_Planner.cwrite("max turning radius 8");
    print_map(std::move(mapFile));

    // assume you've already set up path to cover
    communication_With_Planner.cwrite("path to cover " + to_string(path.get_covered().size()));
    for (point p : path.get_covered())
        communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));

//    cerr << "Waiting for planner to finish starting" << endl;

    // wait for it to finish initializing (?)
    char done[100];
    communication_With_Planner.cread(done, 100);

//    cerr << "Planner is up and running" << endl;

    thread thread_for_planner(thread([=] { requestPath(); }));
    thread_for_planner.detach();

    pause_all = false;

    m_Controller->startSendingControls();
}

void Executive::startController()
{
    m_Controller->startRunning();
    thread thread_for_controller(thread([=] { sendAction(); }));
    thread_for_controller.detach();
}

void Executive::terminate()
{
    running = false;
}

void Executive::finished() {
    pause_all = true;
    terminate();
    m_Controller->stopSendingControls();
    m_Controller->terminate();

    // if the planner isn't dead, kill it
    int status;
    pid_t result = waitpid(communication_With_Planner.getPid(), &status, WNOHANG);
    if (result == 0) {
        kill(communication_With_Planner.getPid(), SIGKILL);
    }

    m_ControlReceiver->allDone();
}
