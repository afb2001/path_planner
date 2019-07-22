#include "DynamicObstaclesManager.h"

double DynamicObstaclesManager::collisionExists(const State &s) {
    return collisionExists(s.x, s.y, s.time);
}

double DynamicObstaclesManager::distanceToNearestPossibleCollision(const State &s) {
    return distanceToNearestPossibleCollision(s.x, s.y, s.speed, s.time);
}

double DynamicObstaclesManager::distanceToNearestPossibleCollision(double x, double y, double speed, double time) {
    return 0;
}

double DynamicObstaclesManager::collisionExists(double x, double y, double time) {
    // Using the complement rule find the probability that there are no collisions
    double prod = 1;
    for (const auto& o : m_Obstacles) {
        auto p = 1 - o.second.probabilityOfCollisionAt(x, y, time);
        prod *= p;
    }
    // then return the complement
    return 1 - prod;
}

void DynamicObstaclesManager::update(uint32_t mmsi, State obstacle) {
    m_Obstacles.at(mmsi).update(obstacle);
}

void DynamicObstaclesManager::update(uint32_t mmsi, const State& obstacle, double stdDev,
                                     double stdDevChangePerSecond) {
    m_Obstacles[mmsi].update(obstacle, stdDev, stdDevChangePerSecond);

}
