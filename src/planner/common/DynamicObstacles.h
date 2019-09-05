#ifndef SRC_DYNAMICOBSTACLES_H
#define SRC_DYNAMICOBSTACLES_H


#include <path_planner/State.h>

class DynamicObstacles {
public:
//    double collisionExists(const double q[3]);
    double collisionExists(const State& s);
    double collisionExists(double x, double y, double time);
    double distanceToNearestPossibleCollision(const double q[3]);
    double distanceToNearestPossibleCollision(const State& s);
    double distanceToNearestPossibleCollision(double x, double y, double speed, double time);
};


#endif //SRC_DYNAMICOBSTACLES_H
