#ifndef SRC_EXECUTIVEINTERNALSMANAGER_H
#define SRC_EXECUTIVEINTERNALSMANAGER_H

#include "path_planner/State.h"
#include <utility>
#include "../planner/utilities/Path.h"
#include <cmath>
#include <vector>
#include <list>
#include <deque>
#include <string>
#include <mutex>
#include <unordered_set>
#include <bitset>
#include <map>

using namespace std;

class ExecutiveInternalsManager
{


  public:
    // Default constructor
    ExecutiveInternalsManager()
    {
        m_Current = m_NextStart = State(0);
        m_Actions.emplace_back(-1);
    };

    ~ExecutiveInternalsManager() = default;

    void replacePath(State &currentLoc);
    //lock this with update info
    void findStart();

    void setNewPath(vector<State> trajectory);

    //below for dynamic obs update
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    void updateCurrent(double x, double y, double speed, double heading, double otime);

    //below for coverd path update
    void updateCovered();

    void addCovered(int x, int y);

    std::vector<State> getActions();

    list<point> getNewlyCovered();

    State getStart();

    bool checkCollision(double cx, double cy, double ex, double ey);

    const list<point> &getCovered() const;

    //below condition check or lock access
    bool finish();

    void initialize();


private:
    std::deque<State> m_Path;
    vector<State> m_Newpath;

    map<uint32_t,State> m_DynamicObstacles;

    list<point> m_Cover, m_Newcover;

    mutex m_PathMutex, m_ToCoverMutex;

    State m_Current, m_NextStart;

    std::vector<State> m_Actions;
};

#endif