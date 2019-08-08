#ifndef SRC_DYNAMICOBSTACLESMANAGER_H
#define SRC_DYNAMICOBSTACLESMANAGER_H


#include <path_planner/State.h>
#include "DynamicObstacle.h"
#include <unordered_map>

class DynamicObstaclesManager {
public:
    double collisionExists(const State& s);
    double collisionExists(double x, double y, double time);
    double distanceToNearestPossibleCollision(const State& s);
    double distanceToNearestPossibleCollision(double x, double y, double speed, double time);

    void add(uint32_t mmsi, const std::vector<Distribution>& distributions, double width, double length);
    void update(uint32_t mmsi, const std::vector<Distribution>& distributions);

private:
    std::unordered_map<uint32_t, DynamicObstacle> m_Obstacles;
};


#endif //SRC_DYNAMICOBSTACLESMANAGER_H
