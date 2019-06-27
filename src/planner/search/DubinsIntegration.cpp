#include <utility>
#include <algorithm>
#include <vector>
#include "DubinsIntegration.h"

int DubinsIntegration::edgeCostCallback(double *q, double accumulatedDistance, void *userData) {
    auto object = static_cast<DubinsIntegration *>(userData);
    if (object->m_ObstacleDistance > DUBINS_INCREMENT) {
        object->m_ObstacleDistance -= DUBINS_INCREMENT;
    } else {
        double staticDistance = object->m_Map->getUnblockedDistance(q[0], q[1]);
        if (staticDistance <= 0) {
            object->m_Penalty += COLLISION_PENALTY;
            object->m_ObstacleDistance = 0;
        } else {
            double dynamicDistance = object->m_DynamicObstacles->distanceToNearestPossibleCollision(q);
            if (dynamicDistance <= 0) {
                object->m_Penalty += object->m_DynamicObstacles->collisionExists(q) * COLLISION_PENALTY;
                object->m_ObstacleDistance = 0;
            } else {
                object->m_ObstacleDistance = fmin(staticDistance, dynamicDistance);
            }
        }
    }
    addNewlyCovered(object->m_ToCover, q[0], q[1], object->m_NewlyCovered);
    return 0;
}

std::vector<std::pair<double, double>>
DubinsIntegration::computeEdgeCollisionPenaltyAndNewlyCovered(DubinsPath path, Map *map, DynamicObstacles *obstacles,
                                                              Path toCover,
                                                              double &penalty) {
    this->m_Map = map;
    this->m_DynamicObstacles = obstacles;
    this->m_ToCover = std::move(toCover);
    this->m_Penalty = 0;
    int err = dubins_path_sample_many(&path, DUBINS_INCREMENT, edgeCostCallback, this);
    if (err != EDUBOK) {
        std::cerr << "Encountered an error in the Dubins library" << std::endl;
    }
    penalty = this->m_Penalty;
    if (!m_NewlyCovered.empty()) {
        std::sort(this->m_NewlyCovered.begin(), this->m_NewlyCovered.end());
        this->m_NewlyCovered.erase(std::unique(this->m_NewlyCovered.begin(), this->m_NewlyCovered.end()), this->m_NewlyCovered.end());
    }
    return this->m_NewlyCovered;
}

void DubinsIntegration::addNewlyCovered(const Path& toCover, double x, double y,
                                        std::vector<std::pair<double, double>> &covered) {
    for (auto p : toCover.get()) {
        if ((p.first - x)*(p.first - x) + (p.second - y)*(p.second - y) < COVERAGE_THRESHOLD * COVERAGE_THRESHOLD) {
            covered.push_back(p);
        }
    }
}

Plan DubinsIntegration::getPlan(const State& start, DubinsPath dubinsPath, double maxSpeed) {
    m_Plan = Plan();
    m_Plan.append(start);
    m_MaxSpeed = maxSpeed;
    int err = dubins_path_sample_many(&dubinsPath, DUBINS_INCREMENT, getPlanCallback, this);
    if (err != EDUBOK) {
        std::cerr << "Encountered an error in the Dubins library" << std::endl;
    }
    return m_Plan;
}

int DubinsIntegration::getPlanCallback(double *q, double accumulatedDistance, void *userData) {
    auto object = static_cast<DubinsIntegration *>(userData);
    object->m_Plan.append(State(q[0], q[1], M_PI_2 - q[2], object->m_MaxSpeed, accumulatedDistance / object->m_MaxSpeed + object->m_Plan.getRef().front().time));
    return 0;
}

DubinsIntegration::DubinsIntegration() = default;
