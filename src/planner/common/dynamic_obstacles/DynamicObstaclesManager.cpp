#include <cfloat>
#include "DynamicObstaclesManager.h"

double DynamicObstaclesManager::collisionExists(const State &s) {
    return collisionExists(s.x, s.y, s.time);
}

double DynamicObstaclesManager::distanceToNearestPossibleCollision(const State &s) {
    return distanceToNearestPossibleCollision(s.x, s.y, s.speed, s.time);
}

double DynamicObstaclesManager::distanceToNearestPossibleCollision(double x, double y, double speed, double time) {
    auto min = DBL_MAX;
    for (const auto& o : m_Obstacles) {
        min = fmin(min, o.second.distanceToEdge(x, y, speed, time));
    }
    return min;
}

double DynamicObstaclesManager::collisionExists(double x, double y, double time) {
    // can't use complement rule because we're not using true probabilities anymore so just use sum of densities
    double sum = 0;
    for (const auto& o : m_Obstacles) {
        sum += o.second.collisionDensityAt(x, y, time);
    }
    return sum;
}

void DynamicObstaclesManager::update(uint32_t mmsi, const std::vector<Distribution>& distributions) {
    auto pair = m_Obstacles.emplace(mmsi, distributions);
    if (!pair.second) pair.first->second.update(distributions);
}

void DynamicObstaclesManager::add(uint32_t mmsi, const std::vector<Distribution>& distributions,
        double width, double length) {
    // hopefully there's nothing already there...
    auto pair = m_Obstacles.insert(std::unordered_map<uint32_t, DynamicObstacle>::value_type(mmsi,
            DynamicObstacle(distributions, length, width)));
}
