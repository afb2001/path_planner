#ifndef OBJECTPAR_H
#define OBJECTPAR_H
#include "string"
#include "iostream"
#include "cmath"

#include "path_planner/StateMsg.h"


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

class State
{

  public:
    double x, y, heading, speed, otime;
    State(double x, double y, double heading, double speed, double otime)
        : x(x), y(y), heading(heading), speed(speed), otime(otime){};
    State(int value)
        : x(value), y(value), heading(value), speed(value), otime(value){};
    State()
        : x(-1), y(-1), heading(-1), speed(-1), otime(-1){};

    void set(double &newx, double &newy, double &newheading, double &newspeed, double &newtime)
    {
        x = newx;
        y = newy;
        heading = newheading;
        speed = newspeed;
        otime = newtime;
    }

    void set(State other)
    {
        x = other.x;
        y = other.y;
        heading = other.heading;
        speed = other.speed;
        otime = other.otime;
    }

    void set(path_planner::StateMsg other)
    {
        x = other.x;
        y = other.y;
        heading = other.heading;
        speed = other.speed;
        otime = other.time;
    }

    void setEstimate(double timeinterval, State &object)
    {
        double displacement = timeinterval * object.speed;
        x = object.x + sin(object.heading) * displacement;
        y = object.y + cos(object.heading) * displacement;
        heading = object.heading;
        speed = object.speed;
        otime = object.otime + 1;
         
    }

    explicit operator path_planner::StateMsg()
    {
        path_planner::StateMsg state;
        state.x = x;
        state.y = y;
        state.heading = heading;
        state.speed = speed;
        state.time = otime;
        return state;
    }

    std::string toString()
    {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading) + " " + std::to_string(speed) + " " + std::to_string(otime);
    }

    std::string toString() const
    {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading) + " " + std::to_string(speed) + " " + std::to_string(otime);
    }

    void print()
    {
        std::cout << x << " " << y << " " << heading << " " << speed << " " << otime << std::endl;
    }

    void printerror()
    {
        std::cerr << x << " " << y << " " << heading << " " << speed << " " << otime << std::endl;
    }
};

#endif