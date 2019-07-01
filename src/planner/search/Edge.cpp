#include <utility>

#include <utility>

#include <utility>
#include <algorithm>
#include <memory>
#include "Edge.h"
#include "../common/Map.h"
#include "../common/DynamicObstacles.h"
#include "../common/Path.h"

extern "C" {
#include <dubins.h>
}

Edge::Edge(std::shared_ptr<Vertex> start) {
    this->m_Start = std::move(start);
}

Edge::Edge(std::shared_ptr<Vertex> start, const State& end) : Edge(std::move(start)){
    setEnd(end);
}

double Edge::computeTrueCost(Map *map, DynamicObstacles *obstacles, const Path& toCover,
                             double maxSpeed, double maxTurningRadius) {
    double collisionPenalty;
    if (m_ApproxCost == -1) computeApproxCost(maxSpeed, maxTurningRadius);
    double& penalty = collisionPenalty;
    std::vector<std::pair<double, double>> result;
    double q[3];
    double lengthSoFar = 0;
    double length = dubins_path_length(&dubinsPath);
    double obstacleDistance = 0;
    std::vector<std::pair<double, double>> newlyCovered1;
    while (lengthSoFar < length) {
        dubins_path_sample(&dubinsPath, lengthSoFar, q);
        if (obstacleDistance > DUBINS_INCREMENT) {
            obstacleDistance -= DUBINS_INCREMENT;
        } else {
            double staticDistance = map->getUnblockedDistance(q[0], q[1]);
            if (staticDistance <= 0) {
                penalty += COLLISION_PENALTY;
                obstacleDistance = 0;
            } else {
                double dynamicDistance = obstacles->distanceToNearestPossibleCollision(q);
                if (dynamicDistance <= 0) {
                    penalty += obstacles->collisionExists(q) * COLLISION_PENALTY;
                    obstacleDistance = 0;
                } else {
                    obstacleDistance = fmin(staticDistance, dynamicDistance);
                }
            }
        }
        for (auto p : toCover.get()) {
            if ((p.first - q[0]) * (p.first - q[0]) + (p.second - q[1]) * (p.second - q[1]) < COVERAGE_THRESHOLD * COVERAGE_THRESHOLD) {
                newlyCovered1.push_back(p);
            }
        }
        lengthSoFar += DUBINS_INCREMENT;

    }
    if (!newlyCovered1.empty()) {
        std::sort(newlyCovered1.begin(), newlyCovered1.end());
        newlyCovered1.erase(std::unique(newlyCovered1.begin(), newlyCovered1.end()), newlyCovered1.end());
    }
    result = newlyCovered1;
    auto newlyCovered = result;
    end()->state().time = start()->state().time + dubins_path_length(&dubinsPath) / maxSpeed;

    end()->uncovered().clear();
    for (auto p : start()->uncovered().get()) {
        if (std::find(newlyCovered.begin(), newlyCovered.end(), p) != newlyCovered.end()) {
            end()->uncovered().add(p);
        }
    }
    m_TrueCost = netTime() * TIME_PENALTY + collisionPenalty;

    // maybe update end's true cost?

    return m_TrueCost;
}

double Edge::computeApproxCost(double maxSpeed, double maxTurningRadius) {
    double q0[3] = {start()->state().x, start()->state().y, start()->state().yaw()};
    double q1[3] = {end()->state().x, end()->state().y, end()->state().yaw()};
    int err = dubins_shortest_path(&dubinsPath, q0, q1, maxTurningRadius);
    if (err != EDUBOK) {
        std::cerr << "Encountered an error in the Dubins library" << std::endl;
    } else {
        m_ApproxCost = dubins_path_length(&dubinsPath) / maxSpeed * TIME_PENALTY;
    }
    return m_ApproxCost;
}

double Edge::netTime() {
    return end()->state().time - start()->state().time;
}

void Edge::smooth(Map* map, DynamicObstacles* obstacles, double maxSpeed, double maxTurningRadius) {
    if (start()->isRoot()) return;
    double parentCost = start()->parentEdge()->m_TrueCost; // should be up to date in A*, check for BIT*
    auto smoothed = Vertex::connect(start()->parent(), end()->state());
    double smoothedCost = smoothed->parentEdge()->computeTrueCost(map, obstacles, start()->uncovered(), maxSpeed, maxTurningRadius);
    if (smoothedCost < parentCost + m_TrueCost && smoothed->approxToGo() <= end()->approxToGo()) {
        // Should be memory-safe, as smoothed will delete the old vertex when it goes out of scope, and all pointers
        // to *end() will still be valid
        // TODO! -- apparently this is actually broken (something throws std::bad_weak_ptr when smoothing runs)
        std::swap(*(smoothed.get()), *(end().get()));
    }
}

Plan Edge::getPlan(double maxSpeed) {
    double q[3];
    double lengthSoFar = 0;
    double length = dubins_path_length(&dubinsPath);
    Plan plan1;
    while (lengthSoFar < length) {
        dubins_path_sample(&dubinsPath, lengthSoFar, q);
        plan1.append(State(q[0], q[1], M_PI_2 - q[2], maxSpeed, lengthSoFar / maxSpeed + start()->state().time));
        lengthSoFar += DUBINS_INCREMENT;
    }
    return plan1;
}

std::shared_ptr<Vertex> Edge::setEnd(const State &state) {
    auto ptr = std::make_shared<Vertex>(state, std::shared_ptr<Edge>(this));
    m_End = ptr;
    return ptr;
}

std::shared_ptr<Vertex> Edge::start() {
    return m_Start;
}

std::shared_ptr<Vertex> Edge::end() {
    std::shared_ptr<Vertex> s(m_End);
    return s;
}

Edge::~Edge() = default;
