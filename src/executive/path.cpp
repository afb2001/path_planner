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

        double angle = atan2(objectPar.y - next_start.y, objectPar.x - next_start.x);
        double displacement = (objectPar.time - next_start.time) * objectPar.speed;
        double diffx = objectPar.x + displacement * cos(angle) - next_start.x;
        double diffy = objectPar.y + displacement * sin(angle) - next_start.y;
        if (debug)
            diffx = diffy = 0;
        for (auto i : newpath)
            if (i.time > objectPar.time)
                path.emplace_back(i.x + diffx, i.y + diffy, i.heading, i.speed, i.time);

//        if (!debug)
//            path.insert(path.end(), newpath.begin(), newpath.end());
    }
    newpath.clear();
};
//lock this with update info
void ExecutiveInternalsManager::findStart()
{
    mtx_path.lock();
    State current_loc = current;
    int index = 0;
    replacePath(current_loc);
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

void ExecutiveInternalsManager::setNewPath(vector<State> trajectory)
{
    mtx_path.lock();
    newpath = std::move(trajectory);
    mtx_path.unlock();
}

void ExecutiveInternalsManager::updateDynamicObstacle(uint32_t mmsi, State obstacle)
{
    dynamic_obstacles[mmsi] = obstacle;
}

void ExecutiveInternalsManager::update_current(double x, double y, double speed, double heading, double otime)
{
    current.x = x;
    current.y = y;
    current.speed = speed;
    current.heading = heading;
    current.time = otime;
}

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
}

vector<State> ExecutiveInternalsManager::getActions()
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
        ret.push_back ((!cover.empty()) ? State(-1) : State(-2));
    }
    mtx_path.unlock();
    return ret;
}

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
    actions.clear();
    next_start = current;
    actions.push_back(current);
    next_start.time += 1;
}

list<point> ExecutiveInternalsManager::getNewlyCovered() {
    return newcover;
}

State ExecutiveInternalsManager::getStart() {
    findStart();
    return next_start;
}
