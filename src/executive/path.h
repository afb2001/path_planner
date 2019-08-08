#include <utility>

#ifndef __PATH_H__
#define __PATH_H__

#include "path_planner/State.h"
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
        current = next_start = State(0);
        actions.emplace_back(-1);
    };

    ~ExecutiveInternalsManager() = default;

    void replacePath(State &currentLoc);
    //lock this with update info
    void findStart();

    void setNewPath(vector<State> trajectory);

    //below for dynamic obs update
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    void update_current(double x, double y, double speed, double heading, double otime);

    //below for coverd path update
    void update_covered();

    void add_covered(int x, int y);

    std::vector<State> getActions();

    list<point> getNewlyCovered();

    State getStart();

    bool checkCollision(double cx, double cy, double ex, double ey);

    const list<point> &get_covered() const;

    //below condition check or lock access
    bool finish();

    void initialize();

    bool debug;
    

  private:
    std::deque<State> path;
    vector<State> newpath;

    map<uint32_t,State> dynamic_obstacles;

    list<point> cover, newcover;

    mutex mtx_path, mtx_cover;

    State current, next_start;

    int dummy;

    std::vector<State> actions;
};

#endif