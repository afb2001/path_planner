#include <utility>

#ifndef __PATH_H__
#define __PATH_H__

#include "path_planner/State.h"
#include "../planner/utilities/Path.h"

#include <cmath>
#include <vector>
#include <list>
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
        pathindex = 0;
        current = next_start = State(0);
        actions[0] = State(-1);
    };

    ~ExecutiveInternalsManager(){
        delete [] Obstacles;
    };

    void replacePath(State &objectPar);
    //lock this with update info
    void findStart();

    //below for reading the path from planner
    void update_newpath(char currentString[], double &bound);

    void setNewPath(vector<State> trajectory);

    //below for dynamic obs update
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    //below for current location update
    void update_current(const char currentString[], int byte);

    void update_current(double x, double y, double speed, double heading, double otime);

    //below for coverd path update
    void update_covered();

    void add_covered(int x, int y);

    //below construct string for controler
    void construct_path_string(string &s);

    void sendAction(string &s, int &sleep);

    /**
     * Returns an array of length 5 containing the current location and
     * the first four states of the plan. The consumer of this array must
     * free it.
     * @return array of length 5
     */
    State* getActions();

    //below construct the request string
    void get_newcovered(string &s);

    list<point> getNewlyCovered();

    State getStart();

    void getDynamicObs(string &s);

    string construct_request_string();

    bool checkCollision(double cx, double cy, double ex, double ey);

    //below access method for executive
//    const vector<State> &getDynamicObs() const;
//
//    const vector<State> &getPath() const;
//
//    const State &getNext() const;

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
    

  private:
    vector<State> path;
    vector<State> newpath;
//    vector<State> dyamic_obstacles;

    map<uint32_t,State> dynamic_obstacles;

    list<point> cover, newcover;

    string defaultAction_1 = State(-1).toString();
    string defaultAction_2 = State(-2).toString();

    mutex mtx_path, mtx_obs, mtx_cover;

    State current, next_start;

    int pathindex, dummy, byteREAD, dummyindex;

    State actions[4];
    string initialVariance = " 1.7";
    double tempx, tempy, tempspeed, temptime, tempheading;

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