#ifndef OBJECTPAR_H
#define OBJECTPAR_H

//#include <robust_dubins/RobustDubins_Problem.h>
//#include <robust_dubins/RobustDubins.h>
#include "string"
#include "iostream"
#include "cmath"

//#include "path_planner/StateMsg.h"


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

/**
 * This class represents a state of the vehicle in five dimensions: x, y, heading, speed, time.
 * TODO! -- add expected units
 */
class State
{

  public:
    // x, y in meters, heading in radians east of north, speed in m/s, time in seconds
    double x, y, heading, speed, time;
    /**
 * Construct a State.
 * @param x
 * @param y
 * @param heading
 * @param speed
 * @param t
 */
    State(double x, double y, double heading, double speed, double t)
        : x(x), y(y), heading(heading), speed(speed), time(t){};
    State(int value)
        : x(value), y(value), heading(value), speed(value), time(value){};
    State()
        : x(-1), y(-1), heading(-1), speed(-1), time(-1){};
//    State(const path_planner::StateMsg& other)
//        : State(other.x, other.y, other.heading, other.speed, other.time) {};

    void set(double &newx, double &newy, double &newheading, double &newspeed, double &newtime)
    {
        x = newx;
        y = newy;
        heading = newheading;
        speed = newspeed;
        time = newtime;
    }

    void set(State other)
    {
        x = other.x;
        y = other.y;
        heading = other.heading;
        speed = other.speed;
        time = other.time;
    }

//    void set(path_planner::StateMsg other)
//    {
//        x = other.x;
//        y = other.y;
//        heading = other.heading;
//        speed = other.speed;
//        time = other.time;
//    }

    void setEstimate(double timeInterval, const State& object)
    {
        double displacement = timeInterval * object.speed;
        x = object.x + sin(object.heading) * displacement;
        y = object.y + cos(object.heading) * displacement;
        heading = object.heading;
        speed = object.speed;
        time = object.time + timeInterval;
         
    }

//    explicit operator path_planner::StateMsg()
//    {
//        path_planner::StateMsg state;
//        state.x = x;
//        state.y = y;
//        state.heading = heading;
//        state.speed = speed;
//        state.time = time;
//        return state;
//    }

    /**
     * Get the score of another state.
     *
     * This is meant for doing MPC.
     *
     * Note: squared distance
     *
     * TODO! -- should include heading distance
     */
    double getDistanceScore(const State &other) const
    {
        double timeDistance = time - other.time;
        double headingDistance = fabs(fmod((heading - other.heading), 2 * M_PI) / 4);
        double speedDifference = fabs(speed - other.speed) / 2;
        double displacement = timeDistance * other.speed;
        double dx = x - (other.x + sin(other.heading)*displacement);
        double dy = y - (other.y + cos(other.heading)*displacement);
        return (dx * dx + dy * dy) + headingDistance + speedDifference; // how do you score headings??
    }

//    std::string toString()
//    {
//        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading) + " " + std::to_string(speed) + " " + std::to_string(time);
//    }

    std::string toString() const
    {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading*180/M_PI) + " " + std::to_string(speed) + " " + std::to_string(time);
    }

    void print()
    {
        std::cout << x << " " << y << " " << heading << " " << speed << " " << time << std::endl;
    }

    void printerror()
    {
        std::cerr << x << " " << y << " " << heading << " " << speed << " " << time << std::endl;
    }

    // from the Go version of State:

    double headingTo(const State& other) const {
        return headingTo(other.x, other.y);
    }

    double headingTo(const std::pair<double, double> p) const {
        return headingTo(p.first, p.second);
    }

    double headingTo(double x1, double y1) const {
        double dx = x1 - this->x;
        double dy = y1 - this->y;
        double h = M_PI_2 - atan2(dy, dx); // TODO! -- is this correct?
        if (h < 0) h += 2 * M_PI;
        return h;
    }

    void setHeadingTowards(const State& other) {
        heading = headingTo(other);
        if (heading < 0) heading += 2 * M_PI;
    }

    void setHeadingTowards(double x1, double y1) {
        heading = headingTo(x1, y1);
        if (heading < 0) heading += 2 * M_PI;
    }

    /**
     * Heading north of east. Val called it "yaw".
     * @return
     */
    double yaw() const {
        double h = M_PI_2 - heading;
        if (h < 0) h += 2 * M_PI;
        return h;
    }

    double timeUntil(const State& other) const {
        return other.time - time;
    }

    inline bool operator==(const State& rhs) const {
        return x == rhs.x &&
                y == rhs.y &&
                heading == rhs.heading &&
                speed == rhs.speed &&
                time == rhs.time;
    }

    bool colocated(const State& rhs) const {
        return x == rhs.x &&
               y == rhs.y &&
               heading == rhs.heading;
    }

    double distanceTo(const State& other) const {
        return distanceTo(other.x, other.y);
    }

    double distanceTo(double x1, double y1) const {
        return sqrt((this->x - x1)*(this->x - x1) + (this->y - y1)*(this->y - y1));
    }

//    double dubinsDistanceTo(double x2, double y2, double yaw2, double turningRadius) const {
//        RobustDubins::Problem problem;
//        problem.set_stateInitial(x, y, yaw());
//        problem.set_stateFinal(x2, y2, yaw2);
//        problem.set_minTurningRadius(turningRadius);
//        RobustDubins::Solver solver;
//        solver.set_problemStatement(problem);
//        solver.solve();
//        return solver.get_optimalPath().get_cost();
//    }
//
//    double dubinsDistanceTo(const State& other, double turningRadius) const {
//        return dubinsDistanceTo(other.x, other.y, other.yaw(), turningRadius);
//    }
};

#endif