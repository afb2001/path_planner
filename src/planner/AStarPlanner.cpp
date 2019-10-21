#include "AStarPlanner.h"
#include <utility>

using std::shared_ptr;

AStarPlanner::AStarPlanner(double maxSpeed, double maxTurningRadius, shared_ptr<Map> staticMap) : SamplingBasedPlanner(
        maxSpeed, maxTurningRadius, std::move(staticMap)) {}

std::function<bool(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)> AStarPlanner::getVertexComparator() {
    return [] (const shared_ptr<Vertex>& v1, const shared_ptr<Vertex>& v2) {
        return v1->f() > v2->f();
    };
}

std::vector<State> AStarPlanner::plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                                      DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
    m_PointsToCover.remove(newlyCovered);
    return plan(start, dynamicObstacles, timeRemaining);
}

std::vector<State> AStarPlanner::plan(const RibbonManager& ribbonManager, const State& start,
                                      DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
    setRibbonManager(ribbonManager);
    return plan(start, dynamicObstacles, timeRemaining);
}

std::vector<State> AStarPlanner::plan(const State& start, DynamicObstaclesManager dynamicObstacles,
                                      double timeRemaining) {
    double endTime = timeRemaining + now();
    m_ExpandedCount = 0;
    m_StartStateTime = start.time;
    m_Samples.clear();
    double minX, maxX, minY, maxY, minSpeed = m_MaxSpeed, maxSpeed = m_MaxSpeed;
    double magnitude = m_MaxSpeed * Plan::timeHorizon();
    minX = start.x - magnitude;
    maxX = start.x + magnitude;
    minY = start.y - magnitude;
    maxY = start.y + magnitude;
    StateGenerator generator = m_UseRibbons? StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7, m_RibbonManager)
                                           : StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7); // lucky seed
    auto startV = m_UseRibbons? Vertex::makeRoot(start, m_RibbonManager) : Vertex::makeRoot(start, m_PointsToCover);
    startV->computeApproxToGo();
    shared_ptr<Vertex> bestVertex(nullptr);
    auto ribbonSamples = m_RibbonManager.findStatesOnRibbonsOnCircle(start, m_CoverageTurningRadius * 2 + 1);
//    if (m_UseRibbons) {
//        for (const auto& s : m_RibbonManager.findStatesOnRibbonsOnCircle(start, m_CoverageTurningRadius * 2 + 1)) {
//            m_Samples.push_back(s);
//        }
//    }
    while (now() < endTime) {
        clearVertexQueue();
        pushVertexQueue(startV);
        // manually expand starting node to include states on nearby ribbons far enough away such that the
        expandToCoverSpecificSamples(startV, ribbonSamples, &dynamicObstacles);
        // On the first iteration add INITIAL_SAMPLES samples, otherwise just double them
        if (m_Samples.size() < c_InitialSamples) addSamples(generator, c_InitialSamples);
        else addSamples(generator);
        auto v = aStar(&dynamicObstacles, endTime);
        if (!bestVertex || (v && v->f() < bestVertex->f())) {
//            if (v) *m_Output << "Found a plan with final fvalue " << v->f() << std::endl;
//            else *m_Output << "Returned from A* with no plan" << std::endl;
            // found a (better) plan
            bestVertex = v;
        }
    }
    *m_Output << m_Samples.size() << " total samples, " << m_ExpandedCount << " expanded" << std::endl;
    if (!bestVertex) {
        *m_Output << "Failed to find a plan" << std::endl;
        return std::vector<State>();
    } else {
//        *m_Output << "Best Plan " << bestVertex->ribbonManager().dumpRibbons() << std::endl; // "Best Plan Ribbons: "
        return tracePlan(bestVertex, false, &dynamicObstacles).get();
    }
}

shared_ptr<Vertex> AStarPlanner::aStar(DynamicObstaclesManager* obstacles, double endTime) {
    auto vertex = popVertexQueue();
    while (now() < endTime) {
        if (goalCondition(vertex)) {
//            *m_Output << "Found goal: " << vertex->toString() << std::endl;
            return vertex;
        }
//        *m_Output << "Expanding vertex at " << vertex->state().toString() << std::endl;
        expand(vertex, obstacles);

        // should probably check if vertex queue is empty but expand should always push some on

        vertex = popVertexQueue();
    }
    return shared_ptr<Vertex>(nullptr);
}

AStarPlanner::AStarPlanner(double maxSpeed, double maxTurningRadius, double coverageSpeed, double coverageTurningRadius,
                           std::shared_ptr<Map> staticMap) : AStarPlanner(maxSpeed, maxTurningRadius, std::move(staticMap))
                           {
    m_CoverageMaxSpeed = coverageSpeed;
    m_CoverageTurningRadius = coverageTurningRadius;
}

void AStarPlanner::expandToCoverSpecificSamples(Vertex::SharedPtr root, const std::vector<State>& samples, DynamicObstaclesManager *obstacles) {
    if (m_CoverageTurningRadius > 0) {
        for (auto s : samples) {
            s.speed = m_CoverageMaxSpeed;
            auto destinationVertex = Vertex::connect(root, s, m_CoverageTurningRadius, true);
            destinationVertex->parentEdge()->computeTrueCost(m_Map, obstacles);
            pushVertexQueue(destinationVertex);
        }
    }
}
