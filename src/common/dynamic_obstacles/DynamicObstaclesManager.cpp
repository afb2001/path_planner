#include <cfloat>
#include "DynamicObstaclesManager.h"
#include <algorithm>

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
    // we're not using true probabilities anymore so just use sum of densities
    double sum = 0;
    for (const auto& o : m_Obstacles) {
        sum += o.second.collisionDensityAt(x, y, time);
    }
    return sum;
}

void DynamicObstaclesManager::update(uint32_t mmsi, const std::vector<Distribution>& distributions) {
    if (std::find(m_IgnoreList.begin(), m_IgnoreList.end(), mmsi) != m_IgnoreList.end()) return;
    auto pair = m_Obstacles.emplace(mmsi, distributions);
    if (!pair.second) pair.first->second.update(distributions);
}

void DynamicObstaclesManager::add(uint32_t mmsi, const std::vector<Distribution>& distributions,
        double width, double length) {
    if (std::find(m_IgnoreList.begin(), m_IgnoreList.end(), mmsi) != m_IgnoreList.end()) return;
    // hopefully there's nothing already there...
    auto pair = m_Obstacles.insert(std::unordered_map<uint32_t, DynamicObstacle>::value_type(mmsi,
            DynamicObstacle(distributions, length, width)));
}

void DynamicObstaclesManager::forget(uint32_t mmsi) {
    m_Obstacles.erase(mmsi);
}

void DynamicObstaclesManager::addIgnore(uint32_t mmsi) {
    m_IgnoreList.push_back(mmsi);
}

void DynamicObstaclesManager::removeIgnore(uint32_t mmsi) {
    m_IgnoreList.erase(std::remove(m_IgnoreList.begin(), m_IgnoreList.end(), 8), m_IgnoreList.end());
}
