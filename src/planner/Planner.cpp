#include <memory>

#include "Planner.h"
#include "search/Vertex.h"
#include <algorithm>    // std::remove_if, std::reverse
#include <utility>

using std::vector;
using std::pair;
using std::shared_ptr;

//Planner::Planner() = default;

//void Planner::addToCover(const vector<pair<double, double>>& points) {
//    m_PointsToCover.add(points);
//}
//
//void Planner::clearToCover() {
//    m_PointsToCover.clear();
//}

//std::vector<State> Planner::plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
//                                 DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
//    m_PointsToCover.remove(newlyCovered);
//    // point to point plan
//    auto cur = Vertex::makeRoot(start, m_PointsToCover); // root
//    shared_ptr<Vertex> prev(nullptr);
//    vector<State> pointsWithHeadings; // points to cover with headings
//    for (auto & p : m_PointsToCover.get()) {
//        pointsWithHeadings.emplace_back(p.first, p.second, 0, m_MaxSpeed, 0);
//        if (pointsWithHeadings.size() > 1) {
//            (pointsWithHeadings.end() - 2)->setHeadingTowards(pointsWithHeadings.back());
//        }
//    }
//    for (auto p : pointsWithHeadings) {
//        prev = cur;
//        cur = Vertex::connect(cur, p);
//        cur->parentEdge()->computeTrueCost(m_Map, &dynamicObstacles, m_MaxSpeed, m_TurningRadius);
//    }
//    auto p = tracePlan(cur, false, &dynamicObstacles);
//    return p.get();
//}

Plan Planner::tracePlan(const shared_ptr<Vertex>& v, bool smoothing, const DynamicObstaclesManager& obstacles) {
    vector<shared_ptr<Edge>> branch;
    if (!v) {
        return Plan();
    }
//    if (smoothing) {
//        v->parentEdge()->smooth(m_Config.map(), obstacles, m_MaxSpeed, m_TurningRadius);
//    }
    for (auto cur = v; !cur->isRoot(); cur = cur->parent()) {
        branch.push_back(cur->parentEdge());
    }

    Plan plan;
    for (auto it = branch.rbegin(); it != branch.rend(); it++) {
        plan.append((*it)->getPlan(m_Config));
        plan.append((*it)->end()->state());
    }
    plan.append(v->state());
    return plan;
}

double Planner::now() const {
    return m_Config.now();
}

//void Planner::updateMap(Map::SharedPtr map) {
//    m_Map = map;
//}

//std::vector<State> Planner::plan(const RibbonManager& ribbonManager, const State& start,
//                                 DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
//    throw std::runtime_error("Ribbon point-to-point planner is not yet implemented");
//}

std::vector<State> Planner::plan(const RibbonManager& ribbonManager, const State& start,
                                 PlannerConfig config, double timeRemaining) {
    m_Config = std::move(config);
    throw std::runtime_error("Ribbon point-to-point planner is not yet implemented");
}

Planner::Planner() : m_Config(PlannerConfig(&std::cerr)) {}

void Planner::setConfig(PlannerConfig config) {
    m_Config = std::move(config);
}

//void Planner::setK(int k) {}
