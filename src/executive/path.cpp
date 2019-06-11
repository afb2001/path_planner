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
//        if (Obstacles[getIndex((int)newx,(int)newy)])
//            return true;
//        cx = newx - ex;
//        cy = newy - ey;
//    }
//    return false;
}

void Path::updateAndAdjustPath(State &currentLocation)
{
    if (!newPath.empty())
    {
        path.clear();

//        double angle = atan2(currentLocation.y - next_start.y, currentLocation.x - next_start.x);
//        double displacement = (currentLocation.time - next_start.time) * currentLocation.speed;
//        double diffx = currentLocation.x + displacement * cos(angle) - next_start.x;
//        double diffy = currentLocation.y + displacement * sin(angle) - next_start.y;
//        if (debug)
//            diffx = diffy = 0;
        for (auto i : newPath) {
            if (i.time > currentLocation.time) {
//                path.emplace_back(i.x + diffx, i.y + diffy, i.heading, i.speed, i.time);
//                diffx /= 2;
//                diffy /= 2;
                path.emplace_back(i.x, i.y, i.heading, i.speed, i.time);
            }
        }

//        if (!debug)
//            path.insert(path.end(), newPath.begin(), newPath.end());
    } else {
//        cerr << "newPath is empty" << endl;
    }
//    newPath.clear();
}
//lock this with update info
void Path::findStart()
{
    mtx_path.lock();
    State current_loc = current;
    int index = 0;
    updateAndAdjustPath(current_loc);
    actions.clear();

    if (!path.empty() && path.back().time > 1 + current_loc.time)
    {
        for (int i = 0; i < path.size(); i++)
        {

            if (i + 1 < path.size() && checkCollision(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y)) {
                cerr << "i + 1 < path size " << (i+1 < path.size()) << endl;
                cerr << "COLLISION " << endl;
            }
            
            if (path[i].time > current_loc.time)
            {
//                path[i].heading = fmod(path[i].heading + 10000 * M_PI, 2 * M_PI);
                actions.push_back(path[i]);
//                if(index == 0)
//                {
                    // TODO! -- this gets out of sync with reality
//                    next_start = path[i];
//                    cerr << "next start heading: " << next_start.heading << endl;
//                }
                index++;

            }
        }
    } else {
        if (path.empty()) cerr << "path is empty." << endl;
        else cerr << "path is in the past. If we're just starting this is fine" << endl;
//        next_start.setEstimate(1, current_loc);
        actions.emplace_back(-1);
    }
    mtx_path.unlock();
}

State Path::readStateFromPlanner(char *currentString)
{
    double x, y, heading, speed, t;
    sscanf(currentString, "%lf %lf %lf %lf %lf\n", &x, &y, &heading, &speed, &t);
    auto s = State(x, y, heading, speed, t); // not sure why but this breaks if I don't do this
    return s;
}

void Path::setNewPath(vector<State> trajectory)
{
    mtx_path.lock();
    newPath = std::move(trajectory);
    mtx_path.unlock();
}

void Path::updateDynamicObstacle(uint32_t mmsi, State obstacle)
{
    dynamic_obstacles[mmsi] = obstacle;
}

void Path::update_current(double x, double y, double speed, double heading, double t)
{
    current.x = x;
    current.y = y;
    current.speed = speed;
    current.heading = heading;
    current.time = t;
}

//below for coverd path update
void Path::update_covered()
{
    mtx_cover.lock();
    auto it = toCover.begin();
    while (it != toCover.end())
    {
        float x = it->x - current.x;
        float y = it->y - current.y;
        if (x * x + y * y <= 20)
        {
            auto it1 = it;
            newlyCovered.push_back(*it);
            ++it;
            toCover.erase(it1);
        }
        else
            ++it;
    }
    mtx_cover.unlock();
}

void Path::add_covered(int x, int y)
{
    toCover.emplace_back(x, y);
}

vector<State> Path::getActions()
{
    vector<State> ret;
    findStart(); // trying this out
    mtx_path.lock(); // should be quick
    ret.push_back(current);
    if (!actions.empty() && actions[0].time > current.time)
    {
        // ditch states already in the past
        while (!path.empty() && current.time > path.front().time)
            path.pop_front();
        if (path.empty()) {
            // All states in the trajectory are in the past
            // TODO! -- appropriate error handling for this case
            cerr << "All the states the planner gave us are in the past" << endl;
        } else {
            for (auto s : actions) ret.push_back(s);
        }
    }
    else
    {
        if (actions.empty()) cerr << "actions is empty" << endl;
        ret.push_back ((!toCover.empty()) ? State(-1) : State(-2));
    }
    mtx_path.unlock();
    return ret;
}

//below construct the request string
void Path::get_newcovered(string &s)
{
    mtx_cover.lock();
    int size = newlyCovered.size();
    s += "newly covered " + to_string(size);
    for (point p : newlyCovered)
        s += "\n" + p.toString();
    newlyCovered.clear();
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

string Path::construct_request_string(State startState)
{
    next_start = startState;
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



const list<point> &Path::getToCover() const
{
    return toCover;
}

//below condition check or lock access
bool Path::finish()
{
    if (toCover.empty())
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
    next_start.time += 1;
}