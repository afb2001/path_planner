#include <tuple>        // std::forward_as_tuple
#include "BinaryDynamicObstaclesManager.h"

double BinaryDynamicObstaclesManager::collisionExists(double x, double y, double time) const {
    double sum = 0;
    for (auto o : m_Obstacles) {
        // not using references in loop statement because this mutates them
        auto& obstacle = o.second;
        obstacle.project(time);
        auto translatedX = x - obstacle.X;
        auto translatedY = y - obstacle.Y;
        auto rotatedX = translatedX * cos(obstacle.Yaw) - translatedY * sin(obstacle.Yaw);
        auto rotatedY = translatedX * sin(obstacle.Yaw) + translatedY * cos(obstacle.Yaw);
        if (fabs(rotatedX) < obstacle.Length / 2 && fabs(rotatedY) < obstacle.Width / 2) sum++;
    }
//    if (sum > 0) std::cerr << "Collision \"probability\" non-zero" << std::endl;
    return sum;
}

void BinaryDynamicObstaclesManager::update(uint32_t mmsi, double x, double y, double heading, double speed, double time,
        double width, double length) {
    if (!isIgnored(mmsi)) m_Obstacles.emplace(std::piecewise_construct,
            std::forward_as_tuple(mmsi),
            std::forward_as_tuple(x, y, heading, speed, time, width, length)
            );
}

void BinaryDynamicObstaclesManager::forget(uint32_t mmsi) {
    m_Obstacles.erase(mmsi);
}
