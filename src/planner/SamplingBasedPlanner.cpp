#include "SamplingBasedPlanner.h"
#include <algorithm>

SamplingBasedPlanner::SamplingBasedPlanner(double maxSpeed, double maxTurningRadius, const Map& staticMap) : Planner(
        maxSpeed, maxTurningRadius, staticMap) {}

std::vector<State> SamplingBasedPlanner::plan(const std::vector<std::pair<double, double>>& newlyCovered,
                                              const State& start, DynamicObstacles dynamicObstacles) {
    m_StartStateTime = start.time;
    m_Samples.clear();
    m_VertexQueue.clear();
    double minX, maxX, minY, maxY, minSpeed = m_MaxSpeed, maxSpeed = m_MaxSpeed;
    double magnitude = m_MaxSpeed * TIME_HORIZON;
    minX = start.x - magnitude;
    maxX = start.x + magnitude;
    minY = start.y - magnitude;
    maxY = start.y + magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7); // lucky seed
    addSamples(generator, 1000);
    std::shared_ptr<Vertex> vertex;
    for (vertex = std::make_shared<Vertex>(start); !goalCondition(vertex); vertex = popVertexQueue()) {
        expand(vertex, &dynamicObstacles);
    }
    return tracePlan(vertex, false, &dynamicObstacles).get();
}

void SamplingBasedPlanner::pushVertexQueue(const std::shared_ptr<Vertex>& vertex) {
    m_VertexQueue.push_back(vertex);
    std::push_heap(m_VertexQueue.begin(), m_VertexQueue.end(), getVertexComparator());
}

std::shared_ptr<Vertex> SamplingBasedPlanner::popVertexQueue() {
    std::pop_heap(m_VertexQueue.begin(), m_VertexQueue.end(), getVertexComparator());
    auto ret = m_VertexQueue.back();
    m_VertexQueue.pop_back();
    return ret;
}

std::function<bool(std::shared_ptr<Vertex> v1,
                   std::shared_ptr<Vertex> v2)> SamplingBasedPlanner::getVertexComparator() {
    return [](std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2){
        return v1->getDepth() < v2->getDepth();
    };
}

std::function<bool(const State& s1, const State& s2)> SamplingBasedPlanner::getStateComparator(const State& origin) {
    return [&origin](const State& s1, const State& s2) {
        return s1.distanceTo(origin) < s2.distanceTo(origin); // use Dubins distance?
    };
}

bool SamplingBasedPlanner::goalCondition(const std::shared_ptr<Vertex>& vertex) {
    return vertex->state().time > m_StartStateTime + TIME_HORIZON ||
            (vertex->uncovered().size() == 0 && vertex->state().time > m_StartStateTime + TIME_MINIMUM);
}

void SamplingBasedPlanner::expand(const std::shared_ptr<Vertex>& sourceVertex, DynamicObstacles* obstacles) {
    // add nearest point to cover
    if (m_PointsToCover.size() != 0) {
        std::pair<double, double> nearest;
        auto minDistance = DBL_MAX;
        for (auto p : m_PointsToCover.get()) {
            auto d = sourceVertex->state().distanceTo(p.first, p.second);
            if (d < minDistance) {
                nearest = p;
            }
        }
        // TODO! -- which heading for points?
        pushVertexQueue(std::make_shared<Vertex>(State(nearest.first, nearest.second, 0, m_MaxSpeed, 0)));
    }
    auto comp = getStateComparator(sourceVertex->state());
    std::make_heap(m_Samples.begin(), m_Samples.end(), comp);
    for (int i = 0; i < k(); i++) {
        auto destinationVertex = Vertex::connect(sourceVertex, m_Samples.front());
        std::pop_heap(m_Samples.begin(), m_Samples.end() - i, comp);
        if (destinationVertex->state() == sourceVertex->state()) {
            i--;
        } else {
            destinationVertex->parentEdge()->computeTrueCost(&m_Map, obstacles, m_MaxSpeed, m_MaxTurningRadius);
            pushVertexQueue(destinationVertex);
        }
    }
}

int SamplingBasedPlanner::k() const {
    return 5;
}

void SamplingBasedPlanner::addSamples(StateGenerator& generator, int n) {
    for (int i = 0; i < n; i++) {
        m_Samples.push_back(generator.generate());
    }
}

void SamplingBasedPlanner::addSamples(StateGenerator& generator) {
    addSamples(generator, m_Samples.size());
}
