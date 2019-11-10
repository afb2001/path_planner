#include "SamplingBasedPlanner.h"
#include <algorithm>
#include <utility>

SamplingBasedPlanner::SamplingBasedPlanner() {}

//std::vector<State> SamplingBasedPlanner::plan(const std::vector<std::pair<double, double>>& newlyCovered,
//                                              const State& start,
//                                              DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
//    m_PointsToCover.remove(newlyCovered);
//    return plan(start, dynamicObstacles, timeRemaining);
//}

//std::vector<State> SamplingBasedPlanner::plan(const RibbonManager& ribbonManager, const State& start,
//                                              DynamicObstaclesManager dynamicObstacles, double timeRemaining) {
//    setRibbonManager(ribbonManager);
//    return plan(start, dynamicObstacles, timeRemaining);
//}

//std::vector<State> SamplingBasedPlanner::plan(const State& start, DynamicObstaclesManager dynamicObstacles,
//                                              double timeRemaining) {
//    m_StartStateTime = start.time;
//    m_Samples.clear();
//    m_VertexQueue.clear();
//    double minX, maxX, minY, maxY, minSpeed = m_Config.maxSpeed(), maxSpeed = m_Config.maxSpeed();
//    double magnitude = m_Config.maxSpeed() * Plan::timeHorizon();
//    minX = start.x - magnitude;
//    maxX = start.x + magnitude;
//    minY = start.y - magnitude;
//    maxY = start.y + magnitude;
//
//    StateGenerator generator = StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7, m_RibbonManager); // lucky seed
//    addSamples(generator, 1000);
//    std::shared_ptr<Vertex> vertex;
//    for (vertex = (Vertex::makeRoot(start, m_RibbonManager));
//         !goalCondition(vertex); vertex = popVertexQueue()) {
//        expand(vertex, m_Config.obstacles());
//    }
//    return tracePlan(vertex, false, m_Config.obstacles()).get();
//}

void SamplingBasedPlanner::pushVertexQueue(Vertex::SharedPtr vertex) {
    if (!vertex->isRoot() && vertex->parentEdge()->infeasible()) return;
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
//        return s1.dubinsDistanceTo(origin, m_TurningRadius) > s2.dubinsDistanceTo(origin, m_TurningRadius);
        return s1.distanceTo(origin) > s2.distanceTo(origin);
    };
}

bool SamplingBasedPlanner::goalCondition(const std::shared_ptr<Vertex>& vertex) {
    return vertex->state().time > m_StartStateTime + Plan::timeHorizon() ||
            (vertex->allCovered() && vertex->state().time > m_StartStateTime + Plan::timeMinimum());
}

void SamplingBasedPlanner::expand(const std::shared_ptr<Vertex>& sourceVertex, const DynamicObstaclesManager& obstacles) {
    
//    std::cerr << "Expanding vertex " << sourceVertex->toString() << std::endl;
    // add nearest point to cover
    if (!sourceVertex->allCovered()) {
        auto s = sourceVertex->getNearestPointAsState();
        s.speed = m_Config.maxSpeed();
        // TODO! -- what heading for points?
        auto destinationVertex = Vertex::connect(sourceVertex, s, m_Config.turningRadius(), false);
        destinationVertex->parentEdge()->computeTrueCost(m_Config.map(), obstacles);
        pushVertexQueue(destinationVertex);
        if (m_Config.coverageMaxSpeed() > 0) {
            s.speed = m_Config.coverageMaxSpeed();
            destinationVertex = Vertex::connect(sourceVertex, s, m_Config.coverageTurningRadius(), true);
            destinationVertex->parentEdge()->computeTrueCost(m_Config.map(), obstacles);
            pushVertexQueue(destinationVertex);
        }
    }
    auto comp = getStateComparator(sourceVertex->state());
    auto dubinsComp = getDubinsComparator(sourceVertex->state());
    // heapify first by Euclidean distance
    std::make_heap(m_Samples.begin(), m_Samples.end(), comp);
    // Use another heap to sort by Dubins distance, skipping the samples which are farther away this time.
    // Making all the vertices adds some allocation overhead but it lets us cache the dubins paths
    std::vector<std::shared_ptr<Vertex>> bestSamples, bestCoverageSamples;
    bool regularDone = false, coverageDone = false;
    if (m_Config.coverageTurningRadius() <= 0) coverageDone = true;
    for (uint64_t i = 0; i < m_Samples.size() && (!regularDone || !coverageDone); i++) {
        auto sample = m_Samples.front();
        std::pop_heap(m_Samples.begin(), m_Samples.end() - i, comp);
        if (!regularDone && (bestSamples.size() < k() ||
            bestSamples.front()->parentEdge()->approxCost() > sample.distanceTo(sourceVertex->state()))) {
            if (!sourceVertex->state().colocated(sample)) {
                // don't force speed to be anything in particular, allowing samples to come with unique speeds
                bestSamples.push_back(Vertex::connect(sourceVertex, sample, m_Config.coverageTurningRadius(), false));
                bestSamples.back()->parentEdge()->computeApproxCost();
                std::push_heap(bestSamples.begin(), bestSamples.end(), dubinsComp);
            }
        } else {
            regularDone = true;
        }
        if (!coverageDone && (bestCoverageSamples.size() < k() ||
            bestCoverageSamples.front()->parentEdge()->approxCost() > sample.distanceTo(sourceVertex->state()))) {
            if (!sourceVertex->state().colocated(sample)) {
                sample.speed = m_Config.coverageMaxSpeed(); // force speed to be coverage speed
                bestCoverageSamples.push_back(Vertex::connect(sourceVertex, sample, m_Config.coverageTurningRadius(), true));
                bestCoverageSamples.back()->parentEdge()->computeApproxCost();
                std::push_heap(bestCoverageSamples.begin(), bestCoverageSamples.end(), dubinsComp);
            }
        } else {
            coverageDone = true;
        }
    }
    // Push the closest K onto the open list
    for (int i = 0; i < k(); i++) {
        if (i >= bestSamples.size()) break;
        auto destinationVertex = bestSamples.front();
        std::pop_heap(bestSamples.begin(), bestSamples.end() - i, dubinsComp);
        destinationVertex->parentEdge()->computeTrueCost(m_Config.map(), obstacles);
        pushVertexQueue(destinationVertex);
    }
    // and again for coverage edges
    for (int i = 0; i < k(); i++) {
        if (i >= bestCoverageSamples.size()) break;
        auto destinationVertex = bestCoverageSamples.front();
        std::pop_heap(bestCoverageSamples.begin(), bestCoverageSamples.end() - i, dubinsComp);
        destinationVertex->parentEdge()->computeTrueCost(m_Config.map(), obstacles);
        pushVertexQueue(destinationVertex);
    }
    m_ExpandedCount++;
}

int SamplingBasedPlanner::k() const {
    return m_Config.branchingFactor();
}

void SamplingBasedPlanner::addSamples(StateGenerator& generator, int n) {
    for (int i = 0; i < n; i++) {
        const auto s = generator.generate();
        m_Samples.push_back(s);
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

//void SamplingBasedPlanner::setRibbonManager(const RibbonManager& ribbonManager) {
//    m_RibbonManager = ribbonManager;
//    m_UseRibbons = true;
//}

std::vector<State> SamplingBasedPlanner::plan(const RibbonManager&, const State& start, PlannerConfig config,
                                              double timeRemaining) {
    m_Config = config;
    m_StartStateTime = start.time;
    m_Samples.clear();
    m_VertexQueue.clear();
    double minX, maxX, minY, maxY, minSpeed = m_Config.maxSpeed(), maxSpeed = m_Config.maxSpeed();
    double magnitude = m_Config.maxSpeed() * Plan::timeHorizon();
    minX = start.x - magnitude;
    maxX = start.x + magnitude;
    minY = start.y - magnitude;
    maxY = start.y + magnitude;

    StateGenerator generator = StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7, m_RibbonManager); // lucky seed
    addSamples(generator, 1000);
    std::shared_ptr<Vertex> vertex;
    for (vertex = Vertex::makeRoot(start, m_RibbonManager);
         !goalCondition(vertex); vertex = popVertexQueue()) {
        expand(vertex, m_Config.obstacles());
    }
    return tracePlan(vertex, false, m_Config.obstacles()).get();
}

//void SamplingBasedPlanner::setK(int k) {
//    m_K = k;
//}


