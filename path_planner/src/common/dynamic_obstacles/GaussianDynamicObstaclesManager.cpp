#include "GaussianDynamicObstaclesManager.h"

double GaussianDynamicObstaclesManager::collisionExists(double x, double y, double time) const {
    double sum = 0;
    for (auto o : m_Obstacles) {
        auto& obstacle = o.second;
        obstacle.project(time);
        sum += obstacle.pdf(Eigen::Vector2d(x, y));
    }
    if (sum < 1e-5) return 0;
    return sum;
}

void GaussianDynamicObstaclesManager::update(uint32_t mmsi, double x, double y, double heading, double speed,
                                             double time) {
    if (!isIgnored(mmsi)) {
        auto result = m_Obstacles.emplace(std::piecewise_construct,
                std::forward_as_tuple(mmsi),
                std::forward_as_tuple(x, y, heading, speed, time));
        if (!result.second) {
            result.first->second = Obstacle(x, y, heading, speed, time);
        }
    }
}

void GaussianDynamicObstaclesManager::forget(uint32_t mmsi) {
    m_Obstacles.erase(mmsi);
}

const std::unordered_map<uint32_t, GaussianDynamicObstaclesManager::Obstacle>& GaussianDynamicObstaclesManager::get() const {
    return m_Obstacles;
}

void GaussianDynamicObstaclesManager::update(uint32_t mmsi, double x, double y, double heading, double speed,
                                             double time, Eigen::Matrix<double, 2, 2> covariance) {
    if (!isIgnored(mmsi)) {
        auto result = m_Obstacles.emplace(std::piecewise_construct,
                                          std::forward_as_tuple(mmsi),
                                          std::forward_as_tuple(x, y, heading, speed, time, covariance));
        if (!result.second) {
            result.first->second = Obstacle(x, y, heading, speed, time, covariance);
        }
    }
}
