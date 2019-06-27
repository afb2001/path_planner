#include <memory>

#include "Planner.h"
#include "search/Vertex.h"
#include <algorithm>    // std::remove_if, std::reverse

using std::vector;
using std::pair;
using std::shared_ptr;

Planner::Planner(double maxSpeed, double maxTurningRadius, Map staticMap) {
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
    cur->uncovered = m_PointsToCover;
    vector<State> pointsWithHeadings; // points to cover with headings
    for (auto it = m_PointsToCover.get().begin(); it != m_PointsToCover.get().end(); it++) {
        pointsWithHeadings.emplace_back(it->first, it->second, 0, m_MaxSpeed, 0);
        if (pointsWithHeadings.size() > 1) {
            (pointsWithHeadings.end() - 2)->setHeadingTowards(pointsWithHeadings.back());
        }
    }
    for (auto p : pointsWithHeadings) {
        prev = cur;
        cur = Vertex::connect(cur, p);
        cur->parentEdge->computeTrueCost(&m_Map, &dynamicObstacles, m_PointsToCover, m_MaxSpeed, m_MaxTurningRadius);
    }
    auto p = tracePlan(cur, false);
    return p.get();
}

Plan Planner::tracePlan(shared_ptr<Vertex> v, bool smoothing) {
    vector<shared_ptr<Edge>> branch;
    if (!v) {
        return Plan();
    }
    if (smoothing) {
        v->parentEdge->smooth();
    }
    for (auto cur = v; !cur->isRoot(); cur = cur->parent()) {
        branch.push_back(cur->parentEdge);
    }

    Plan plan;
    for (auto it = branch.rbegin(); it != branch.rend(); it++) {
        plan.append((*it)->getPlan(m_MaxSpeed));
        plan.append((*it)->end()->state);
    }
    plan.append(v->state);
    return plan;
}
