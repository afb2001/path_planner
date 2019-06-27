#include "path.h"
using namespace std;

bool ExecutiveInternalsManager::checkCollision(double sx, double sy, double ex, double ey)
{
    return false;
    // TODO! -- support negative map coordinates
//    double d = atan2(ey - sy, ex - sx);
//    double newx = sx, newy = sy;
//    double cx = newx - ex;
//    double cy = newy - ey;
//    point p;
//    while (true)
//    {
//        if (cx * cx + cy * cy < 0.09)
//            break;
//        newx += cos(d) * 0.3;
//        newy += sin(d) * 0.3;
//        if (Obstacles[getindex((int)newx,(int)newy)])
//            return true;
//        cx = newx - ex;
//        cy = newy - ey;
//    }
//    return false;
}

void ExecutiveInternalsManager::replacePath(State &objectPar)
{
    if (newpath.size() > 1)
    {
        path.clear();
        pathindex = 0;

        double angle = atan2(objectPar.y - next_start.y, objectPar.x - next_start.x);
        double displacement = (objectPar.time - next_start.time) * objectPar.speed;
        double diffx = objectPar.x + displacement * cos(angle) - next_start.x;
        double diffy = objectPar.y + displacement * sin(angle) - next_start.y;
        if (debug)
            diffx = diffy = 0;
        for (auto i : newpath)
            if (i.time > objectPar.time)
                path.emplace_back(i.x + diffx, i.y + diffy, i.heading, i.speed, i.time);

        if (!debug)
            path.insert(path.end(), newpath.begin(), newpath.end());
    }
    newpath.clear();
};
//lock this with update info
void ExecutiveInternalsManager::findStart()
{
    mtx_path.lock();
    int path_size = path.size();
    bool find = false, visit = true;
    State current_loc = current;
    double time_time[4]{current_loc.time + 1,current_loc.time + 2,current_loc.time + 3,current_loc.time + 4};
    int index = 0;
    replacePath(current_loc);

    if (path_size > 1 && pathindex < path_size)
    {
        for (int i = pathindex; i < path_size; i++)
        {

            if (path[i].time <= time_time[0] && i + 1 < path_size  && checkCollision(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y)) {
                cerr << "time less than time_time" << (path[i].time <= time_time[0]) << endl;
                cerr << "i + 1 < path size " << (i+1 < path_size) << endl;
                cerr << "COLLISION " << endl;
            }
            
            if (path[i].time > time_time[index])
            {
                double predictHead = fmod(path[i].heading + 10000 * M_PI, 2 * M_PI);
                actions[index].set(path[i].x, path[i].y, predictHead, path[i].speed, path[i].time);
                if(index == 3)
                {
                    find = true;
                    break;
                }
                else if(index == 0)
                {
                    next_start.set(path[i].x, path[i].y, path[i].heading, path[i].speed, time_time[index]);
                    visit = false;
                }
                index += 1;

            }
        }
    }

    if (!find)
    {
        if (visit)
        {
            next_start.setEstimate(1, current_loc);
        }
        actions[0] = State(-1);
    }
    mtx_path.unlock();
};

//below for reading the path from planner
void ExecutiveInternalsManager::update_newpath(char currentString[], double &bound)
{
    sscanf(currentString, "%lf %lf %lf %lf %lf\n", &tempx, &tempy, &tempheading, &tempspeed, &temptime);
    if (bound < temptime)
        newpath.emplace_back(tempx, tempy, tempheading, tempspeed, temptime);
};

void ExecutiveInternalsManager::setNewPath(vector<State> trajectory)
{
    mtx_path.lock();
    newpath = std::move(trajectory);
    mtx_path.unlock();
}

//below for dynamic obs update
//int Path::update_dynamic_obs(char ObsString[], int byte, int i)
//{
//    if (dyamic_obstacles.size() <= i)
//        dyamic_obstacles.emplace_back();
//    if (sscanf(ObsString + byte, "%d,%lf,%lf,%lf,%lf,%lf\n%n", &dummyindex, &dyamic_obstacles[i].x, &dyamic_obstacles[i].y, &dyamic_obstacles[i].speed, &dyamic_obstacles[i].heading, &dyamic_obstacles[i].time, &byteREAD) == 6)
//        return byteREAD;
//    dyamic_obstacles.pop_back();
//    return 0;
//};

void ExecutiveInternalsManager::updateDynamicObstacle(uint32_t mmsi, State obstacle)
{
    dynamic_obstacles[mmsi] = obstacle;
}

//below for current location update
void ExecutiveInternalsManager::update_current(const char currentString[], int byte)
{
    sscanf(currentString + byte, "%lf,%lf,%lf,%lf,%lf [%d]", &current.x, &current.y, &current.speed, &current.heading, &current.time, &dummy);
};

void ExecutiveInternalsManager::update_current(double x, double y, double speed, double heading, double otime)
{
    current.x = x;
    current.y = y;
    current.speed = speed;
    current.heading = heading;
    current.time = otime;
};

//below for coverd path update
void ExecutiveInternalsManager::update_covered()
{
    mtx_cover.lock();
    auto it = cover.begin();
    while (it != cover.end())
    {
        float x = it->x - current.x;
        float y = it->y - current.y;
        if (x * x + y * y <= 20)
        {
            auto it1 = it;
            newcover.push_back(*it);
            ++it;
            cover.erase(it1);
        }
        else
            ++it;
    }
    mtx_cover.unlock();
}

void ExecutiveInternalsManager::add_covered(int x, int y)
{
    cover.emplace_back(x, y);
};

//below construct string for controler
void ExecutiveInternalsManager::construct_path_string(string &s)
{
    int size = path.size();
    s += "path " + to_string(size - pathindex) + "\n";
    for (int i = pathindex; i < size; i++)
        s += path[i].toString() + '\n';
    s += next_start.toString() + '\0'; //for estimate start
};

void ExecutiveInternalsManager::sendAction(string &s, int &sleep)
{
    s = "";
    sleep = 50;
    State current_loc = current;
    mtx_path.lock();
    int path_size = path.size();
    if (actions[0].time > current_loc.time)
    {
        while (path_size > pathindex && current_loc.time > path[pathindex].time)
            pathindex++;
        if (path_size == pathindex)
            return;

        s += current_loc.toString() + "\n";
        for(int i = 0; i< 4; i++)
            s += actions[i].toString() + "\n";
        construct_path_string(s);
    }
    else
    {
        s += current_loc.toString() + "\n";
        s += (!cover.empty()) ? defaultAction_1 : defaultAction_2;
        s += "\npath 0\n" + next_start.toString() + '\0';
    }
    mtx_path.unlock();
}

State* ExecutiveInternalsManager::getActions()
{
    auto* ret = new State[5]; // consumer frees
    mtx_path.lock(); // should be quick
    State current_loc = current;
    ret[0] = current_loc;
    int path_size = path.size();
    if (actions[0].time > current_loc.time)
    {
        while (path_size > pathindex && current_loc.time > path[pathindex].time)
            pathindex++;
        if (path_size == pathindex) {
            // All states in the trajectory are in the past
            // TODO! -- appropriate error handling for this case
            delete[] ret;
            return nullptr;
        }

        for(int i = 0; i< 4; i++)
            ret[i+1] = actions[i];
    }
    else
    {
        ret[1] = (!cover.empty()) ? State(-1) : State(-2);
    }
    mtx_path.unlock();
    return ret;
}

//below construct the request string
void ExecutiveInternalsManager::get_newcovered(string &s)
{
    mtx_cover.lock();
    int size = newcover.size();
    s += "newly covered " + to_string(size);
    for (point p : newcover)
        s += "\n" + p.toString();
    newcover.clear();
    mtx_cover.unlock();
};

void ExecutiveInternalsManager::getDynamicObs(string &s)
{
    mtx_obs.lock();
    int dynamic_obs_size = dynamic_obstacles.size();
    s += "dynamic obs " + to_string(dynamic_obs_size);
    for (auto & obstacle : dynamic_obstacles)
        s += "\n" + to_string(obstacle.first) + " " + obstacle.second.toString();// + initialVariance;
    mtx_obs.unlock();
}

string ExecutiveInternalsManager::construct_request_string()
{
    string s = "plan\n";
    get_newcovered(s);
    findStart();
    s += "\nstart state " + next_start.toString() + "\n";
    getDynamicObs(s);
    return s;
}

//below access method for executive
//const vector<State> &Path::getDynamicObs() const
//{
//    return dyamic_obstacles;
//};
//
//const vector<State> &Path::getPath() const
//{
//    return path;
//};
//
//const State &Path::getNext() const
//{
//    return next_start;
//};

const State &ExecutiveInternalsManager::getCurrent() const
{
    return current;
};



const list<point> &ExecutiveInternalsManager::get_covered() const
{
    return cover;
};

//below condition check or lock access
bool ExecutiveInternalsManager::finish()
{
    if (cover.empty())
    {
        actions[0] = State(-2);
        return true;
    }
    return false;
}

void ExecutiveInternalsManager::initialize()
{
    actions[0] = next_start = current;
    next_start.time += 1;
}

void ExecutiveInternalsManager::lock_obs()
{
    mtx_obs.lock();
};

void ExecutiveInternalsManager::unlock_obs()
{
    mtx_obs.unlock();
}

list<point> ExecutiveInternalsManager::getNewlyCovered() {
    return newcover;
}

State ExecutiveInternalsManager::getStart() {
    findStart();
    return next_start;
}
