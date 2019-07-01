#include <memory>

#include "Planner.h"
#include "search/Vertex.h"
#include <algorithm>    // std::remove_if, std::reverse

using std::vector;
using std::pair;
using std::shared_ptr;

Planner::Planner(double maxSpeed, double maxTurningRadius, const Map& staticMap) {
    m_MaxSpeed = maxSpeed;
    m_MaxTurningRadius = maxTurningRadius;
    m_Map = staticMap;
}

void Planner::addToCover(const vector<pair<double, double>>& points) {
    m_PointsToCover.add(points);
}

void Planner::clearToCover() {
    m_PointsToCover.clear();
}

std::vector<State> Planner::plan(const vector<pair<double, double>> &newlyCovered, const State &start,
                                 DynamicObstacles dynamicObstacles) {
    m_PointsToCover.remove(newlyCovered);
    // point to point plan
    auto cur = std::make_shared<Vertex>(start); // root
    shared_ptr<Vertex> prev(nullptr);
    cur->setUncovered(m_PointsToCover);
    vector<State> pointsWithHeadings; // points to cover with headings
    for (auto & p : m_PointsToCover.get()) {
        pointsWithHeadings.emplace_back(p.first, p.second, 0, m_MaxSpeed, 0);
        if (pointsWithHeadings.size() > 1) {
            (pointsWithHeadings.end() - 2)->setHeadingTowards(pointsWithHeadings.back());
        }
    }
    for (auto p : pointsWithHeadings) {
        prev = cur;
        cur = Vertex::connect(cur, p);
        cur->parentEdge()->computeTrueCost(&m_Map, &dynamicObstacles, m_MaxSpeed, m_MaxTurningRadius);
    }
    auto p = tracePlan(cur, false, &dynamicObstacles);
    return p.get();
}

Plan Planner::tracePlan(const shared_ptr<Vertex>& v, bool smoothing, DynamicObstacles* obstacles) {
    vector<shared_ptr<Edge>> branch;
    if (!v) {
        return Plan();
    }
    if (smoothing) {
        v->parentEdge()->smooth(&m_Map, obstacles, m_MaxSpeed, m_MaxTurningRadius);
    }
    for (auto cur = v; !cur->isRoot(); cur = cur->parent()) {
        branch.push_back(cur->parentEdge());
    }

    Plan plan;
    for (auto it = branch.rbegin(); it != branch.rend(); it++) {
        plan.append((*it)->getPlan(m_MaxSpeed));
        plan.append((*it)->end()->state());
    }
    plan.append(v->state());
    return plan;
}
