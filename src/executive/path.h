#ifndef __PATH_H__
#define __PATH_H__

#include "path_planner/State.h"
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <mutex>
#include <unordered_set>
#include <bitset>
#include <deque>

// region point

struct point{
    int x, y;
    point()
            :x(0),y(0) {}

    point(int x,int y)
            :x(x),y(y) {}

    bool operator==(const point &s) const
    {
        return x == s.x && y == s.y;
    }

    std::string toString()
    {
        return std::to_string(x) + " " + std::to_string(y);
    }
};

// endregion

using namespace std;

class Path
{
  public:
    // Default constructor
    Path()
    {
        current = next_start = State(0);
    };

    ~Path(){
        delete [] Obstacles;
    };

    /**
     * Update m_Path from newPath, adjusting to where we actually are in time.
     * @param currentLocation our current location
     */
    void updateAndAdjustPath(State &currentLocation);

    //below for reading the path from planner
    static State readStateFromPlanner(char *currentString);

    //below for dynamic obs update
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    void update_current(double x, double y, double speed, double heading, double t);

    //below for covered path update
    void update_covered();

    void add_covered(int x, int y);

    /**
     * @return a vector containing the reference trajectory for the controller
     */
    vector<State> getActions();

    string construct_request_string(State startState);

    const State &getCurrent() const;

    const list<point> &getToCover() const;

    int getIndex(int x, int y)
    {
        return y * maxX + x;
    };

    bool finish();

    void initialize();

    int maxX = 0;
    bool *Obstacles = nullptr;

    vector<State> newPath;

    void setNewPath(vector<State> trajectory);

private:
    //lock this with update info
    void findStart();

    //below construct the request string
    void get_newcovered(string &s);

    void getDynamicObs(string &s);

    bool checkCollision(double cx, double cy, double ex, double ey);

    deque<State> path;

    map<uint32_t,State> dynamic_obstacles;

    list<point> toCover, newlyCovered;

    mutex mtx_path, mtx_obs, mtx_cover;

    State current, next_start;

    vector<State> actions;
};

#endif