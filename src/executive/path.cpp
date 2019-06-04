#include <utility>

#include "path.h"
using namespace std;

bool Path::checkCollision(double sx, double sy, double ex, double ey)
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

void Path::updateAndAdjustPath(State &currentLocation)
{
    if (newpath.size() > 1)
    {
        path.clear();

        double angle = atan2(currentLocation.y - next_start.y, currentLocation.x - next_start.x);
        double displacement = (currentLocation.otime - next_start.otime) * currentLocation.speed;
        double diffx = currentLocation.x + displacement * cos(angle) - next_start.x;
        double diffy = currentLocation.y + displacement * sin(angle) - next_start.y;
        if (debug)
            diffx = diffy = 0;
        for (auto i : newpath)
            if (i.otime > currentLocation.otime)
                path.emplace_back(i.x + diffx, i.y + diffy, i.heading, i.speed, i.otime);

//        if (!debug)
//            path.insert(path.end(), newpath.begin(), newpath.end());
    } else {
        cerr << "newpath is empty" << endl;
    }
    newpath.clear();
}
//lock this with update info
void Path::findStart()
{
    mtx_path.lock();
    State current_loc = current;
    int index = 0;
    updateAndAdjustPath(current_loc);
    actions.clear();

    if (!path.empty() && path.back().otime > 1 + current_loc.otime)
    {
        for (int i = 0; i < path.size(); i++)
        {

            if (i + 1 < path.size() && checkCollision(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y)) {
                cerr << "i + 1 < path size " << (i+1 < path.size()) << endl;
                cerr << "COLLISION " << endl;
            }
            
            if (path[i].otime > current_loc.otime)
            {
//                path[i].heading = fmod(path[i].heading + 10000 * M_PI, 2 * M_PI);
                actions.push_back(path[i]);
                if(index == 0)
                {
                    next_start = path[i];
                    // TODO! -- this gets out of sync with reality
//                    cerr << "next start heading: " << next_start.heading << endl;
                }
                index++;

            }
        }
    } else {
        cerr << "path is in the past. If we're just starting this is fine" << endl;
        next_start.setEstimate(1, current_loc);
        actions.emplace_back(-1);
    }
    mtx_path.unlock();
}

State Path::readStateFromPlanner(char *currentString, double &bound)
{
    double x, y, heading, speed, t;
    sscanf(currentString, "%lf %lf %lf %lf %lf\n", &x, &y, &heading, &speed, &t);
    auto s = State(x, y, heading, speed, t); // not sure why but this breaks if I don't do this
    if (bound < t)
        return s;
}

void Path::setNewPath(vector<State> trajectory)
{
    mtx_path.lock();
    newpath = std::move(trajectory);
    mtx_path.unlock();
}

void Path::updateDynamicObstacle(uint32_t mmsi, State obstacle)
{
    dynamic_obstacles[mmsi] = obstacle;
}

//below for current location update
void Path::update_current(const char currentString[], int byte)
{
    sscanf(currentString + byte, "%lf,%lf,%lf,%lf,%lf [%d]", &current.x, &current.y, &current.speed, &current.heading, &current.otime, &dummy);
}

void Path::update_current(double x, double y, double speed, double heading, double otime)
{
    current.x = x;
    current.y = y;
    current.speed = speed;
    current.heading = heading;
    current.otime = otime;
}

//below for coverd path update
void Path::update_covered()
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

void Path::add_covered(int x, int y)
{
    cover.emplace_back(x, y);
}

vector<State> Path::getActions()
{
    vector<State> ret;
    mtx_path.lock(); // should be quick
    ret.push_back(current);
    if (!actions.empty() && actions[0].otime > current.otime)
    {
        // ditch states already in the past
        while (!path.empty() && current.otime > path.front().otime)
            path.pop_front();
        if (path.empty()) {
            // All states in the trajectory are in the past
            // TODO! -- appropriate error handling for this case
            cerr << "All the states the planner gave us are in the past" << endl;
            mtx_path.unlock();
            return ret;
        }
        for (auto s : actions) ret.push_back(s);

    }
    else
    {
        if (actions.empty()) cerr << "actions is empty" << endl;
        ret.push_back ((!cover.empty()) ? State(-1) : State(-2));
    }
    mtx_path.unlock();
    return ret;
}

//below construct the request string
void Path::get_newcovered(string &s)
{
    mtx_cover.lock();
    int size = newcover.size();
    s += "newly covered " + to_string(size);
    for (point p : newcover)
        s += "\n" + p.toString();
    newcover.clear();
    mtx_cover.unlock();
}

void Path::getDynamicObs(string &s)
{
    mtx_obs.lock();
    int dynamic_obs_size = dynamic_obstacles.size();
    s += "dynamic obs " + to_string(dynamic_obs_size);
    for (auto & obstacle : dynamic_obstacles)
        s += "\n" + to_string(obstacle.first) + " " + obstacle.second.toString();// + initialVariance;
    mtx_obs.unlock();
}

string Path::construct_request_string()
{
    string s = "plan\n";
    get_newcovered(s);
    findStart();
    s += "\nstart state " + next_start.toString() + "\n";
    getDynamicObs(s);
    return s;
}

const State &Path::getCurrent() const
{
    return current;
}



const list<point> &Path::get_covered() const
{
    return cover;
}

//below condition check or lock access
bool Path::finish()
{
    if (cover.empty())
    {
        actions.emplace_back(-2);
        return true;
    }
    return false;
}

void Path::initialize()
{
    actions.clear();
    next_start = current;
    actions.push_back(current);
    next_start.otime += 1;
}

void Path::lock_obs()
{
    mtx_obs.lock();
}

void Path::unlock_obs()
{
    mtx_obs.unlock();
}