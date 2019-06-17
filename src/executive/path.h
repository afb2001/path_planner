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

/**
 * This class does a lot of internal stuff for the Executive, namely managing the most recent plan, the static and
 * dynamic obstacles, and the current location of the vehicle.
 */
class Path
{
  public:
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

    /**
     * Parse a state from a C-style string.
     * @param currentString the string
     * @return a state based on that string
     */
    static State readStateFromPlanner(char *currentString);

    /**
     * Update the dynamic obstacle mmsi with a new observation.
     * @param mmsi the dynamic obstacle's mmsi
     * @param obstacle the new observation of the obstacle
     */
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    /**
     * Update the state of the vehicle.
     * @param x
     * @param y
     * @param speed
     * @param heading
     * @param t
     */
    void update_current(double x, double y, double speed, double heading, double t);

    /**
     * Check whether we've covered any new points.
     */
    void update_covered();

    /**
     * Add a point to cover at <x, y>.
     * @param x
     * @param y
     */
    void add_covered(int x, int y);

    /**
     * @return a vector containing the reference trajectory for the controller
     */
    std::vector<State> getActions();

    /**
     * Build a string to send to the planner to ask for a plan from startState.
     * @param startState the desired starting state
     * @return a string for the planner
     */
    std::string construct_request_string(State startState);

    /**
     * Get the current state of the vehicle.
     * @return the current state of the vehicle
     */
    const State &getCurrent() const;

    /**
     * Get the list of points to cover.
     * @return the list of points to cover
     */
    const std::list<point> &getToCover() const;

    /**
     * Get an index into the map corresponding to the grid cell at (x,y).
     * @param x
     * @param y
     * @return an index into the map
     */
    int getIndex(int x, int y)
    {
        return y * maxX + x;
    };

    /**
     * @return whether we've covered all the points or not
     */
    bool isFinished();

    /**
     * Reset some things.
     */
    void initialize();

    int maxX = 0;
    bool *Obstacles = nullptr;

    std::vector<State> newPath;

    /**
     * Set the new path, presumably coming from the planner.
     * @param trajectory the new plan
     */
    void setNewPath(std::vector<State> trajectory);

private:
    //lock this with update info
    void findStart();

    //below construct the request string
    void get_newcovered(std::string &s);

    void getDynamicObs(std::string &s);

    bool checkCollision(double cx, double cy, double ex, double ey);

    std::deque<State> path;

    std::map<uint32_t,State> dynamic_obstacles;

    std::list<point> toCover, newlyCovered;

    std::mutex mtx_path, mtx_obs, mtx_cover;

    State current, next_start;

    std::vector<State> actions;
};

#endif