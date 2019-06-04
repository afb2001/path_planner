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
    static State readStateFromPlanner(char *currentString, double &bound);


    //below for dynamic obs update
//    int update_dynamic_obs(char ObsString[], int byte, int i);

void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    //below for current location update
    void update_current(const char currentString[], int byte);

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

    const list<point> &get_covered() const;

    int getindex(int x,int y)
    {
        return y * Maxx + x;
    };

    //below condition check or lock access
    bool finish();

    void initialize();
    void lock_obs();
    void unlock_obs();

    int Maxx = 0;
    bool *Obstacles = 0;
    bool debug;


    vector<State> newpath;

    void setNewPath(vector<State> trajectory);

private:
    deque<State> path;
//    vector<State> dyamic_obstacles;

    map<uint32_t,State> dynamic_obstacles;

    list<point> cover, newcover;

    string defaultAction_1 = State(-1).toString();
    string defaultAction_2 = State(-2).toString();

    mutex mtx_path, mtx_obs, mtx_cover;

    State current, next_start;

    int dummy;

    vector<State> actions;
    string initialVariance = " 1.7";


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