#include "AStarPlanner.h"

#define INITIAL_SAMPLES 32

using std::shared_ptr;

AStarPlanner::AStarPlanner(double maxSpeed, double maxTurningRadius, const Map& staticMap) : SamplingBasedPlanner(
        maxSpeed, maxTurningRadius, staticMap) {}

std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> AStarPlanner::getVertexComparator() {
    return [] (std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2) {
        return v1->f() > v2->f();
    };
}

std::vector<State> AStarPlanner::plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                                      DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
    double endTime = timeRemaining + now();
    m_PointsToCover.remove(newlyCovered);
    m_ExpandedCount = 0;
    m_StartStateTime = start.time;
    m_Samples.clear();
    double minX, maxX, minY, maxY, minSpeed = m_MaxSpeed, maxSpeed = m_MaxSpeed;
    double magnitude = m_MaxSpeed * TIME_HORIZON;
    minX = start.x - magnitude;
    maxX = start.x + magnitude;
    minY = start.y - magnitude;
    maxY = start.y + magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7); // lucky seed
    auto startV = Vertex::makeRoot(start, m_PointsToCover);
    shared_ptr<Vertex> bestVertex(nullptr);
    while (now() < endTime) {
        clearVertexQueue();
        pushVertexQueue(startV);
        // On the first iteration add INITIAL_SAMPLES samples, otherwise just double them
        if (m_Samples.empty()) addSamples(generator, INITIAL_SAMPLES);
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
    return tracePlan(bestVertex, false, &dynamicObstacles).get();
}

std::shared_ptr<Vertex> AStarPlanner::aStar(DynamicObstaclesManager* obstacles, double endTime) {
    auto vertex = popVertexQueue();
    while (now() < endTime) {
        if (goalCondition(vertex)) {
            return vertex;
        }
//        *m_Output << "Expanding vertex at " << vertex->state().toString() << std::endl;
        expand(vertex, obstacles);

        // should probably check if vertex queue is empty but expand should always push some on

        vertex = popVertexQueue();
    }
    return std::shared_ptr<Vertex>(nullptr);
}
