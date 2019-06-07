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

namespace std
{

    template <>
    struct hash<point>
    {
        size_t operator()(const point &c) const
        {
            int x = c.x, y = c.y;
            unsigned long value = 0;
            for (int i = 0; i < 4; i++)
            {
                if (!x)
                    break;
                value += value * 31 + (x & 8);
                x = x >> 8;
            }
            for (int i = 0; i < 4; i++)
            {
                if (!y)
                    break;
                value += value * 31 + (y & 8);
                y = y >> 8;
            }
            return value;
        }
    };
}

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
     * Update <code>path</code> from <code>newpath</code>, adjusting to where we actually are.
     * @param currentLocation our current location
     */
    void updateAndAdjustPath(State &currentLocation);

    //lock this with update info
    void findStart();

    //below for reading the path from planner
    static State readStateFromPlanner(char *currentString);


    //below for dynamic obs update
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    void update_current(double x, double y, double speed, double heading, double otime);

    //below for coverd path update
    void update_covered();

    void add_covered(int x, int y);

    /**
     * Returns an array of length 5 containing the current location and
     * the first four states of the plan. The consumer of this array must
     * free it.
     * @return array of length 5
     */
    vector<State> getActions();

    //below construct the request string
    void get_newcovered(string &s);

    void getDynamicObs(string &s);

    string construct_request_string();

    bool checkCollision(double cx, double cy, double ex, double ey);

    const State &getCurrent() const;

    const list<point> &getToCover() const;

    int getindex(int x,int y)
    {
        return y * maxX + x;
    };

    bool finish();

    void initialize();

    int maxX = 0;
    bool *Obstacles = nullptr;
    bool debug;


    vector<State> newpath;

    void setNewPath(vector<State> trajectory);

private:
    deque<State> path;
//    vector<State> dyamic_obstacles;

    map<uint32_t,State> dynamic_obstacles;

    list<point> toCover, newcover;

    mutex mtx_path, mtx_obs, mtx_cover;

    State current, next_start;

    int dummy;

    vector<State> actions;

    //function
    double estimate_x(double timeiterval, State &object)
    {
        return object.x + timeiterval * sin(object.heading) * object.speed;
    };

    double estimate_y(double timeiterval, State &object)
    {
        return object.y + timeiterval * cos(object.heading) * object.speed;
    };

};

#endif