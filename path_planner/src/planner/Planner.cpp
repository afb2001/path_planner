#include <memory>
#include "search/Edge.h"
#include "Planner.h"
#include "search/Vertex.h"
#include <algorithm>    // std::remove_if, std::reverse
#include <utility>

using std::vector;
using std::pair;
using std::shared_ptr;

DubinsPlan Planner::tracePlan(const shared_ptr<Vertex>& v, bool smoothing, const DynamicObstaclesManager& obstacles) {
    vector<shared_ptr<Edge>> branch;
    if (!v) {
        return DubinsPlan();
    }
    bool dangerous = false;
    for (auto cur = v; !cur->isRoot(); cur = cur->parent()) { // extra edge somehow???
        branch.push_back(cur->parentEdge());
        if (cur->parentEdge()->getSavedCollisionPenalty() > 0) {
            std::cerr << "Collision possible in returned plan (penalty = " << cur->parentEdge()->getSavedCollisionPenalty() << ")" << std::endl;
            dangerous = true;
            m_Stats.PlanCollisionPenalty += cur->parentEdge()->getSavedCollisionPenalty();
        }
    }
    DubinsPlan plan;
    plan.setDangerous(dangerous);
    for (auto it = branch.rbegin(); it != branch.rend(); it++) {
        plan.append((*it)->getPlan(m_Config));
    }
    return plan;
}

double Planner::now() const {
    // Pass a function in with the configuration so we can use an exterior time source.
    return m_Config.now();
}

Planner::Stats Planner::plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
                         const DubinsPlan& previousPlan, double timeRemaining) {
    m_Config = std::move(config);
    throw std::runtime_error("Ribbon point-to-point planner is not yet implemented");
}

Planner::Planner() : m_Config(PlannerConfig(&std::cerr)) {}

void Planner::setConfig(PlannerConfig config) {
    m_Config = std::move(config);
}