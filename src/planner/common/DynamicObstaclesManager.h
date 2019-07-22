#ifndef SRC_DYNAMICOBSTACLESMANAGER_H
#define SRC_DYNAMICOBSTACLESMANAGER_H


#include <path_planner/State.h>
#include "DynamicObstacle.h"
#include <unordered_map>

class DynamicObstaclesManager {
public:
//    double collisionExists(const double q[3]);
    double collisionExists(const State& s);
    double collisionExists(double x, double y, double time);
    double distanceToNearestPossibleCollision(const State& s);
    double distanceToNearestPossibleCollision(double x, double y, double speed, double time);

    void update(uint32_t mmsi, State obstacle);
    void update(uint32_t mmsi, const State& obstacle, double stdDev, double stdDevChangePerSecond);

private:
    std::unordered_map<uint32_t, DynamicObstacle> m_Obstacles;
};


#endif //SRC_DYNAMICOBSTACLESMANAGER_H
