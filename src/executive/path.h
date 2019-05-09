#ifndef __PATH_H__
#define __PATH_H__

#include "ObjectPar.h"
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <mutex>
#include <unordered_set>
#include <bitset>

using namespace std;

class Path
{


  public:
    // Default constructor
    Path()
    {
        pathindex = 0;
        current = next_start = ObjectPar(0);
        actions[0] = ObjectPar(-1);
    };

    ~Path(){
        delete [] Obstacles;
    };

    void replacePath(ObjectPar &objectPar);
    //lock this with update info
    void findStart();

    //below for reading the path from planner
    void update_newpath(char currentString[], double &bound);


    //below for dynamic obs update
    int update_dynamic_obs(char ObsString[], int byte, int i);


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
    ObjectPar* getActions();

    //below construct the request string
    void get_newcovered(string &s);

    void getDynamicObs(string &s);

    string construct_request_string();

    bool checkCollision(double cx, double cy, double ex, double ey);

    //below access method for executive
    const vector<ObjectPar> &getDynamicObs() const;

    const vector<ObjectPar> &getPath() const;

    const ObjectPar &getNext() const;

    const ObjectPar &getCurrent() const;

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
    vector<ObjectPar> path;
    vector<ObjectPar> newpath;
    vector<ObjectPar> dyamic_obstacles;

    list<point> cover, newcover;

    string defaultAction_1 = ObjectPar(-1).toString();
    string defaultAction_2 = ObjectPar(-2).toString();

    mutex mtx_path, mtx_obs, mtx_cover;

    ObjectPar current, next_start;

    int pathindex, dummy, byteREAD, dummyindex;

    ObjectPar actions[4];
    string initialVariance = " 1.7";
    double tempx, tempy, tempspeed, temptime, tempheading;

    //function
    double estimate_x(double timeiterval, ObjectPar &object)
    {
        return object.x + timeiterval * sin(object.heading) * object.speed;
    };

    double estimate_y(double timeiterval, ObjectPar &object)
    {
        return object.y + timeiterval * cos(object.heading) * object.speed;
    };
};

#endif