#include <utility>

#include <utility>

#include <utility>
#include <algorithm>
#include <memory>
#include "Edge.h"
#include "../common/Map.h"
#include "../common/DynamicObstacles.h"
#include "DubinsIntegration.h"
#include "../common/Path.h"

Edge::Edge(std::shared_ptr<Vertex> start) {
    this->m_Start = std::move(start);
}

Edge::Edge(std::shared_ptr<Vertex> start, const State& end) : Edge(std::move(start)){
    setEnd(end);
}

double Edge::computeTrueCost(Map *map, DynamicObstacles *obstacles, const Path& toCover,
                             double maxSpeed, double maxTurningRadius) {
    double collisionPenalty;
    DubinsIntegration d;
    if (m_ApproxCost == -1) computeApproxCost(maxSpeed, maxTurningRadius);
    auto newlyCovered = d.computeEdgeCollisionPenaltyAndNewlyCovered(dubinsPath, map, obstacles, toCover,
                                                                     collisionPenalty);
    end()->state.time = start()->state.time + dubins_path_length(&dubinsPath) / maxSpeed;

    end()->uncovered.clear();
    for (auto p : start()->uncovered.get()) {
        if (std::find(newlyCovered.begin(), newlyCovered.end(), p) != newlyCovered.end()) {
            end()->uncovered.add(p);
        }
    }
    m_TrueCost = netTime() * TIME_PENALTY + collisionPenalty;

    // maybe update end's true cost?

    return m_TrueCost;
}

double Edge::computeApproxCost(double maxSpeed, double maxTurningRadius) {
    double q0[3] = {start()->state.x, start()->state.y, start()->state.yaw()};
    double q1[3] = {end()->state.x, end()->state.y, end()->state.yaw()};
    int err = dubins_shortest_path(&dubinsPath, q0, q1, maxTurningRadius);
    if (err != EDUBOK) {
        std::cerr << "Encountered an error in the Dubins library" << std::endl;
    } else {
        m_ApproxCost = dubins_path_length(&dubinsPath) / maxSpeed * TIME_PENALTY;
    }
    return m_ApproxCost;
}

double Edge::netTime() {
    return end()->state.time - start()->state.time;
}

void Edge::smooth() {

}

Plan Edge::getPlan(double maxSpeed) {
    DubinsIntegration d;
    return d.getPlan(start()->state, dubinsPath, maxSpeed);
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
