#include "ExecutiveInternalsManager.h"
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

void ExecutiveInternalsManager::replacePath(State &currentLoc)
{
    if (m_Newpath.size() > 1)
    {
        m_Path.clear();

        double angle = atan2(currentLoc.y - m_NextStart.y, currentLoc.x - m_NextStart.x);
        double displacement = (currentLoc.time - m_NextStart.time) * currentLoc.speed;
        double diffx = currentLoc.x + displacement * cos(angle) - m_NextStart.x;
        double diffy = currentLoc.y + displacement * sin(angle) - m_NextStart.y;
//        if (debug)
            diffx = diffy = 0;
        for (auto i : m_Newpath)
            if (i.time > currentLoc.time)
                m_Path.emplace_back(i.x + diffx, i.y + diffy, i.heading, i.speed, i.time);
            else cerr << "Did not emplace state at time " << i.time << " because the current time is " << currentLoc.time << endl;

//        if (!debug)
//            path.insert(path.end(), m_Newpath.begin(), m_Newpath.end());
    } else {
        cerr << "no new path" << endl;
    }
//    m_Newpath.clear();
}
//lock this with update info
void ExecutiveInternalsManager::findStart()
{
    m_PathMutex.lock();
    State current_loc = m_Current;
    int index = 0;
    replacePath(current_loc);
    m_Actions.clear();

    if (!m_Path.empty() && m_Path.back().time > 1 + current_loc.time)
    {
        for (unsigned long i = 0; i < m_Path.size(); i++)
        {

            if (i + 1 < m_Path.size() && checkCollision(m_Path[i].x, m_Path[i].y, m_Path[i + 1].x, m_Path[i + 1].y)) {
                cerr << "i + 1 < path size " << (i+1 < m_Path.size()) << endl;
                cerr << "COLLISION " << endl;
            }

            if (m_Path[i].time > current_loc.time)
            {
//                path[i].heading = fmod(path[i].heading + 10000 * M_PI, 2 * M_PI);
                m_Actions.push_back(m_Path[i]);
//                if(index == 0)
//                {
                // TODO! -- this gets out of sync with reality
                    m_NextStart = m_Path[i];
//                    cerr << "next start heading: " << m_NextStart.heading << endl;
//                }
                index++;

            }
        }
    } else {
        if (m_Path.empty()) cerr << "path is empty." << endl;
        else cerr << "path is in the past. If we're just starting this is fine" << endl;
        m_NextStart.setEstimate(1, current_loc);
        m_Actions.emplace_back(-1);
    }
    m_PathMutex.unlock();
}

void ExecutiveInternalsManager::setNewPath(vector<State> trajectory)
{
    m_PathMutex.lock();
    m_Newpath = std::move(trajectory);
    m_PathMutex.unlock();
}

void ExecutiveInternalsManager::updateDynamicObstacle(uint32_t mmsi, State obstacle)
{
    m_DynamicObstacles[mmsi] = obstacle;
}

void ExecutiveInternalsManager::updateCurrent(double x, double y, double speed, double heading, double otime)
{
    m_Current.x = x;
    m_Current.y = y;
    m_Current.speed = speed;
    m_Current.heading = heading;
    m_Current.time = otime;
}

//below for coverd path update
void ExecutiveInternalsManager::updateCovered()
{
    m_ToCoverMutex.lock();
    auto it = m_Cover.begin();
    while (it != m_Cover.end())
    {
        auto x = it->x - m_Current.x;
        auto y = it->y - m_Current.y;
        if (x * x + y * y <= 20)
        {
            auto it1 = it;
            m_Newcover.push_back(*it);
            ++it;
            m_Cover.erase(it1);
        }
        else
            ++it;
    }
    m_ToCoverMutex.unlock();
}

void ExecutiveInternalsManager::addCovered(int x, int y)
{
    m_Cover.emplace_back(x, y);
}

vector<State> ExecutiveInternalsManager::getActions()
{

    vector<State> ret;
    findStart(); // trying this out
    m_PathMutex.lock(); // should be quick
    ret.push_back(m_Current);
    if (!m_Actions.empty() && m_Actions[0].time > m_Current.time)
    {
        // ditch states already in the past
        while (!m_Path.empty() && m_Current.time > m_Path.front().time)
            m_Path.pop_front();
        if (m_Path.empty()) {
            // All states in the trajectory are in the past
            // TODO! -- appropriate error handling for this case
            cerr << "All the states the planner gave us are in the past" << endl;
        } else {
            for (auto s : m_Actions) ret.push_back(s);
        }
    }
    else
    {
        if (m_Actions.empty()) cerr << "actions is empty" << endl;
        ret.push_back ((!m_Cover.empty()) ? State(-1) : State(-2));
    }
    m_PathMutex.unlock();
    return ret;
}

const list<point> &ExecutiveInternalsManager::getCovered() const
{
    return m_Cover;
}

//below condition check or lock access
bool ExecutiveInternalsManager::finish()
{
    if (m_Cover.empty())
    {
        m_Actions[0] = State(-2);
        return true;
    }
    return false;
}

void ExecutiveInternalsManager::initialize()
{
    m_Actions.clear();
    m_NextStart = m_Current;
    m_Actions.push_back(m_Current);
    m_NextStart.time += 1;
}

list<point> ExecutiveInternalsManager::getNewlyCovered() {
    return m_Newcover;
}

State ExecutiveInternalsManager::getStart() {
    findStart();
    cerr << "Returning next start = " << m_NextStart.toString() << endl;
    return m_NextStart;
}
