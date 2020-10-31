#include "AStarPlanner.h"
#include "search/Edge.h"
#include <utility>

using std::shared_ptr;

std::function<bool(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)> AStarPlanner::getVertexComparator() {
    return [] (const shared_ptr<Vertex>& v1, const shared_ptr<Vertex>& v2) {
        return v1->f() > v2->f();
    };
}

Planner::Stats AStarPlanner::plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
                              const DubinsPlan& previousPlan, double timeRemaining) {
    m_Config = std::move(config); // gotta do this before we can call now()
    double endTime = timeRemaining + now();
    m_Config.setStartStateTime(start.time());
    m_RibbonManager = ribbonManager;
    m_RibbonManager.changeHeuristicIfTooManyRibbons(); // make sure ribbon heuristic is calculable
    if (m_RibbonManager.done()) m_RibbonManager.setCoverageCompletedTime(start.time());
    m_Stats = Stats();
//    m_ExpandedCount = 0;
    m_IterationCount = 0;
    m_StartStateTime = start.time();
    m_Samples.clear();
    m_AttemptedSamples = 0;
    double minX, maxX, minY, maxY, minSpeed = m_Config.maxSpeed(), maxSpeed = m_Config.maxSpeed();
    double magnitude = m_Config.maxSpeed() * m_Config.timeHorizon();
    auto mapExtremes = m_Config.map()->extremes();
    minX = fmax(start.x() - magnitude, mapExtremes[0]);
    maxX = fmin(start.x() + magnitude, mapExtremes[1]);
    minY = fmax(start.y() - magnitude, mapExtremes[2]);
    maxY = fmin(start.y() + magnitude, mapExtremes[3]);
    auto seed = (unsigned long)endTime; // for different results each time. For consistency, use like 7 or something
    StateGenerator generator = StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, seed, m_RibbonManager); // lucky seed
    auto startV = Vertex::makeRoot(start, m_RibbonManager);
    startV->state().speed() = m_Config.maxSpeed();
    startV->computeApproxToGo(m_Config);
    m_BestVertex = nullptr;
//    auto ribbonSamples = m_RibbonManager.findStatesOnRibbonsOnCircle(start, m_Config.coverageTurningRadius() * 2 + 1);
    std::vector<State> brownPathSamples;
    if (m_Config.useBrownPaths()) {
        brownPathSamples = m_RibbonManager.findNearStatesOnRibbons(start, m_Config.coverageTurningRadius());
    }

    // collision check old plan
    Vertex::SharedPtr lastPlanEnd = startV;
    if (!previousPlan.empty()) {
        for (const auto& p : previousPlan.get()) {
            if (p.getEndTime() <= start.time()) continue;
            if (p.getNetTime() == 0) continue; // There is sometimes a zero length edge at the end. Not sure why
            lastPlanEnd = Vertex::connect(lastPlanEnd, p, p.getRho() == m_Config.coverageTurningRadius());
            lastPlanEnd->parentEdge()->computeTrueCost(m_Config);
            if (lastPlanEnd->parentEdge()->infeasible()) {
                lastPlanEnd = startV;
                break;
            }
            if (goalCondition(lastPlanEnd)) break;
        }
    }
    // big loop
    while (now() < endTime) {
        clearVertexQueue();
        if (m_BestVertex && m_BestVertex->f() <= startV->f()) {
            *m_Config.output() << "Found best possible plan, assuming heuristic admissibility" << std::endl;
            break;
        }
        visualizeVertex(startV, "start", false);

        if (m_Config.visualizations()) {
            // copied here for debugging but it's necessary for visualizing the previous plan
            lastPlanEnd = startV;
            if (!previousPlan.empty()) {
                for (const auto& p : previousPlan.get()) {
                    if (p.getEndTime() <= start.time()) continue;
                    if (p.getNetTime() == 0) continue; // just trying this I guess
                    lastPlanEnd = Vertex::connect(lastPlanEnd, p, p.getRho() == m_Config.coverageTurningRadius());
                    lastPlanEnd->parentEdge()->computeTrueCost(m_Config);
                    lastPlanEnd->computeApproxToGo(m_Config);
                    visualizeVertex(lastPlanEnd, "lastPlanEnd", false);
                    if (lastPlanEnd->parentEdge()->infeasible()) {
                        lastPlanEnd = startV;
                        break;
                    }
                    if (goalCondition(lastPlanEnd)) break;
                }
            }
        }

        if (m_Config.visualizations()) {
            m_Config.visualizationStream() << "Incumbent f-value: " << (m_BestVertex? m_BestVertex->f() : 0) << std::endl;
            m_Config.visualizationStream() << m_RibbonManager.dumpRibbons() << "End Ribbons" << std::endl;
        }
        pushVertexQueue(startV);
        if (lastPlanEnd != startV) pushVertexQueue(lastPlanEnd);
        // manually expand starting node to include states on nearby ribbons far enough away such that the boat doesn't
        // have to loop around

//        expandToCoverSpecificSamples(startV, ribbonSamples, m_Config.obstacles(), true);
        expandToCoverSpecificSamples(startV, brownPathSamples, m_Config.obstaclesManager(), true);
        // On the first iteration add initialSamples samples, otherwise just double them
        if (m_Samples.size() < m_Config.initialSamples()) addSamples(generator, m_Config.initialSamples());
        else addSamples(generator); // double samples (BIT* linearly increases them...)
        // visualize all samples each iteration
        if (m_Config.visualizations()) {
            for (const auto& s : m_Samples)
                m_Config.visualizationStream() << "State: (" << s.toStringRad() << "), f: " << 0 << ", g: " << 0 << ", h: " <<
                                           0 << " sample" << std::endl;
        }
        auto v = aStar(m_Config.obstaclesManager(), endTime);
        if (!m_BestVertex || (v && v->f() + 0.0 < m_BestVertex->f())) { // add fudge factor to favor earlier (simpler) plans
            // found a (better) plan
            m_BestVertex = v;
            if (v && m_Config.visualizations()) {
                visualizePlan(tracePlan(v, false, m_Config.obstaclesManager()));
                visualizeVertex(v, "goal", false);
            }
        }
        m_Stats.Iterations++;
    }
    // Add expected final cost, total accrued cost (not here)
    m_Stats.Samples = m_Samples.size();
    if (!m_BestVertex) {
        *m_Config.output() << "Failed to find a plan" << std::endl;
    } else {
        m_Stats.PlanFValue = m_BestVertex->f();
        m_Stats.PlanDepth = m_BestVertex->getDepth();
        m_Stats.PlanTimePenalty = (m_BestVertex->state().time() - m_StartStateTime) * Edge::timePenaltyFactor();
        m_Stats.PlanHValue = m_BestVertex->approxToGo();
        m_Stats.Plan = std::move(tracePlan(m_BestVertex, false, m_Config.obstaclesManager()));
    }
    return m_Stats;
}

shared_ptr<Vertex> AStarPlanner::aStar(const DynamicObstaclesManager& obstacles, double endTime) {
    auto vertex = popVertexQueue();
    while (now() < endTime) {
        // relying on the filter on the vertex queue to give us a better goal
        if (goalCondition(vertex)) {
            visualizeVertex(vertex, "vertex", false);
            return vertex;
        }
        expand(vertex, obstacles);

        if (vertexQueueEmpty()) return Vertex::SharedPtr(nullptr);
        vertex = popVertexQueue();
    }
    return shared_ptr<Vertex>(nullptr);
}

void AStarPlanner::expandToCoverSpecificSamples(Vertex::SharedPtr root, const std::vector<State>& samples,
                                                const DynamicObstaclesManager& obstacles, bool coverageAllowed) {
    if (m_Config.coverageTurningRadius() > 0) {
        for (auto s : samples) {
            for (const auto& speed : {m_Config.maxSpeed(), m_Config.slowSpeed()}) {
                s.speed() = speed;
                auto destinationVertex = Vertex::connect(root, s, m_Config.coverageTurningRadius(), coverageAllowed);
                destinationVertex->parentEdge()->computeTrueCost(m_Config);
                pushVertexQueue(destinationVertex);
            }
        }
    }
}


