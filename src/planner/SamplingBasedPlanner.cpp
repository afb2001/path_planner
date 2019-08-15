#include "SamplingBasedPlanner.h"
#include <algorithm>
#include <utility>

SamplingBasedPlanner::SamplingBasedPlanner(double maxSpeed, double maxTurningRadius, std::shared_ptr<Map> staticMap) : Planner(
        maxSpeed, maxTurningRadius, std::move(staticMap)) {}

std::vector<State> SamplingBasedPlanner::plan(const std::vector<std::pair<double, double>>& newlyCovered,
                                              const State& start,
                                              DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
    m_StartStateTime = start.time;
    m_Samples.clear();
    m_VertexQueue.clear();
    double minX, maxX, minY, maxY, minSpeed = m_MaxSpeed, maxSpeed = m_MaxSpeed;
    double magnitude = m_MaxSpeed * Plan::timeHorizon();
    minX = start.x - magnitude;
    maxX = start.x + magnitude;
    minY = start.y - magnitude;
    maxY = start.y + magnitude;
    StateGenerator generator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7); // lucky seed
    addSamples(generator, 1000);
    std::shared_ptr<Vertex> vertex;
    for (vertex = Vertex::makeRoot(start, m_PointsToCover); !goalCondition(vertex); vertex = popVertexQueue()) {
        expand(vertex, &dynamicObstacles);
    }
    return tracePlan(vertex, false, &dynamicObstacles).get();
}

void SamplingBasedPlanner::pushVertexQueue(const std::shared_ptr<Vertex>& vertex) {
    m_VertexQueue.push_back(vertex);
    std::push_heap(m_VertexQueue.begin(), m_VertexQueue.end(), getVertexComparator());
}

std::shared_ptr<Vertex> SamplingBasedPlanner::popVertexQueue() {
    if (m_VertexQueue.empty()) throw std::out_of_range("Trying to pop an empty vertex queue");
    std::pop_heap(m_VertexQueue.begin(), m_VertexQueue.end(), getVertexComparator());
    auto ret = m_VertexQueue.back();
    m_VertexQueue.pop_back();
    return ret;
}

std::function<bool(std::shared_ptr<Vertex> v1,
                   std::shared_ptr<Vertex> v2)> SamplingBasedPlanner::getVertexComparator() {
    return [](const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2){
        return v1->getDepth() < v2->getDepth();
    };
}

std::function<bool(const State& s1, const State& s2)> SamplingBasedPlanner::getStateComparator(const State& origin) {
    return [&](const State& s1, const State& s2) {
//        return s1.dubinsDistanceTo(origin, m_MaxTurningRadius) > s2.dubinsDistanceTo(origin, m_MaxTurningRadius);
        return s1.distanceTo(origin) > s2.distanceTo(origin);
    };
}

bool SamplingBasedPlanner::goalCondition(const std::shared_ptr<Vertex>& vertex) {
    return vertex->state().time > m_StartStateTime + Plan::timeHorizon() ||
            (vertex->uncovered().size() == 0 && vertex->state().time > m_StartStateTime + Plan::timeHorizon());
}

void SamplingBasedPlanner::expand(const std::shared_ptr<Vertex>& sourceVertex, DynamicObstaclesManager* obstacles) {
//    std::cerr << "Expanding vertex " << sourceVertex->state().toString() << std::endl;
    // add nearest point to cover
    if (sourceVertex->uncovered().size() != 0) {
        std::pair<double, double> nearest = sourceVertex->getNearestPoint();
        // TODO! -- what heading for points?
        auto destinationVertex = Vertex::connect(sourceVertex, State(nearest.first, nearest.second, 0, m_MaxSpeed, 0));
        destinationVertex->parentEdge()->computeTrueCost(m_Map, obstacles, m_MaxSpeed, m_MaxTurningRadius);
        pushVertexQueue(destinationVertex);
    }
    auto comp = getStateComparator(sourceVertex->state());
    auto dubinsComp = getDubinsComparator(sourceVertex->state());
    // heapify first by Euclidean distance
    std::make_heap(m_Samples.begin(), m_Samples.end(), comp);
    // Use another heap to sort by Dubins distance, skipping the samples which are farther away this time.
    // Making all the vertices adds some allocation overhead but it lets us cache the dubins paths
    std::vector<std::shared_ptr<Vertex>> bestSamples;
    for (uint64_t i = 0; i < m_Samples.size(); i++) {
        auto sample = m_Samples.front();
        std::pop_heap(m_Samples.begin(), m_Samples.end() - i, comp);
        if (bestSamples.size() < k() ||
            bestSamples.front()->parentEdge()->approxCost() > sample.distanceTo(sourceVertex->state())) {
            if (!sourceVertex->state().colocated(sample)) {
                bestSamples.push_back(Vertex::connect(sourceVertex, sample));
                bestSamples.back()->parentEdge()->computeApproxCost(m_MaxSpeed, m_MaxTurningRadius);
                std::push_heap(bestSamples.begin(), bestSamples.end(), dubinsComp);
            }
        } else {
            break;
        }
    }
    // Push the closest K onto the open list
    for (int i = 0; i < k(); i++) {
        if (i >= bestSamples.size()) break;
        auto destinationVertex = bestSamples.front();
        std::pop_heap(bestSamples.begin(), bestSamples.end() - i, dubinsComp);
        destinationVertex->parentEdge()->computeTrueCost(m_Map, obstacles, m_MaxSpeed, m_MaxTurningRadius);
        pushVertexQueue(destinationVertex);
    }
    m_ExpandedCount++;
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

void SamplingBasedPlanner::clearVertexQueue() {
    m_VertexQueue.clear();
}

std::function<bool(const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2)> SamplingBasedPlanner::getDubinsComparator(
        const State& origin) {
    return [&] (const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2) {
        // backwards from other state comparator because we want a max heap not a min heap
        return v1->parentEdge()->approxCost() < v2->parentEdge()->approxCost();
    };
}
