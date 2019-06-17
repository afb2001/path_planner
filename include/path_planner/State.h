#ifndef OBJECTPAR_H
#define OBJECTPAR_H
#include "string"
#include "iostream"
#include "cmath"

#include "path_planner/StateMsg.h"

/**
 * This class represents a state of the vehicle in five dimensions: x, y, heading, speed, time.
 */
class State
{

  public:
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
    /**
     * Construct a State from a StateMsg
     * @param other the StateMsg
     */
    explicit State(path_planner::StateMsg other)
            : x(other.x), y(other.y), heading(other.heading), speed(other.speed), time(other.time){}
    State(State const &other)=default;

    void set(double newX, double newY, double newHeading, double newSpeed, double newTime)
    {
        x = newX;
        y = newY;
        heading = newHeading;
        speed = newSpeed;
        time = newTime;
    }

    void set(State other)
    {
        x = other.x;
        y = other.y;
        heading = other.heading;
        speed = other.speed;
        time = other.time;
    }

    void setEstimate(double timeInterval, State &object)
    {
        double displacement = timeInterval * object.speed;
        x = object.x + sin(object.heading) * displacement;
        y = object.y + cos(object.heading) * displacement;
        heading = object.heading;
        speed = object.speed;
        time = object.time + 1;
         
    }

    /**
     * Convert a State to a StateMsg.
     * @return a StateMsg matching the fields of this State.
     */
    explicit operator path_planner::StateMsg()
    {
        path_planner::StateMsg state;
        state.x = x;
        state.y = y;
        state.heading = heading;
        state.speed = speed;
        state.time = time;
        return state;
    }

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

    std::string toString()
    {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading) + " " + std::to_string(speed) + " " + std::to_string(time);
    }

    std::string toString() const
    {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading) + " " + std::to_string(speed) + " " + std::to_string(time);
    }
};

#endif