#include "SamplingBasedPlanner.h"
#include "search/Edge.h"
#include <algorithm>
#include <utility>

SamplingBasedPlanner::SamplingBasedPlanner() {}

void SamplingBasedPlanner::pushVertexQueue(Vertex::SharedPtr vertex) {
    if (!vertex->isRoot() && vertex->parentEdge()->infeasible()) return;
    vertex->approxToGo(); // make sure it is calculated
    // prune vertices worse than the incumbent solution
    if (m_BestVertex && m_BestVertex->f() < vertex->f()) return; // assumes heuristic is admissible and consistent
    // make sure this isn't a goal with equal f to the incumbent
    if (m_BestVertex && m_BestVertex->f() == vertex->f() && goalCondition(vertex)) return;
    m_VertexQueue.push_back(vertex);
    std::push_heap(m_VertexQueue.begin(), m_VertexQueue.end(), getVertexComparator());
//    std::cerr << "Pushing to vertex queue: " << vertex->toString() << std::endl;
    visualizeVertex(vertex, "vertex", false);
    m_Stats.Generated++;
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
        return s1.distanceTo(origin) > s2.distanceTo(origin);
    };
}

bool SamplingBasedPlanner::goalCondition(const std::shared_ptr<Vertex>& vertex) {
    auto coverageDoneTime = vertex->ribbonManager().coverageCompletedTime() + m_Config.timeMinimum();
    if (vertex->ribbonManager().coverageCompletedTime() == -1 && vertex->ribbonManager().done()) {
        throw std::runtime_error("Unset coverage completed time but coverage is done");
    }
    auto nonCoverageDoneTime = m_StartStateTime + m_Config.timeHorizon();
    return vertex->state().time() >= nonCoverageDoneTime ||
           (vertex->done() && vertex->state().time() >= coverageDoneTime);
}

void SamplingBasedPlanner::expand(const std::shared_ptr<Vertex>& sourceVertex, const DynamicObstaclesManager& obstacles) {
    
//    std::cerr << "Expanding vertex " << sourceVertex->toString() << std::endl;
    visualizeVertex(sourceVertex, "vertex", true);

    // define configurations
    const auto& speeds = {m_Config.maxSpeed(), m_Config.maxSpeed() == m_Config.slowSpeed()?
                                               -1 : m_Config.slowSpeed()};
    const int nTurningRadii = 2;
    const double turningRadii[nTurningRadii] = {m_Config.turningRadius(),
                                m_Config.coverageTurningRadius() == m_Config.turningRadius()?
                                -1 : m_Config.coverageTurningRadius()};
    // add nearest point to cover
    if (!sourceVertex->done()) {
        auto s = sourceVertex->getNearestPointAsState();
        // TODO! -- get some set of near points
        if (sourceVertex->state().distanceTo(s) > m_Config.collisionCheckingIncrement()) {
            for (const auto& speed : speeds) {
                if (speed <= 0) continue;
                for (const auto& turningRadius : turningRadii) {
                    if (turningRadius <= 0) continue;
                    bool coverageAllowed = turningRadius == m_Config.coverageTurningRadius();
                    s.speed() = speed;
                    auto destinationVertex = Vertex::connect(sourceVertex, s, turningRadius, coverageAllowed);
                    destinationVertex->parentEdge()->computeTrueCost(m_Config);
                    pushVertexQueue(destinationVertex);
                }
            }
        }
    }
    auto comp = getStateComparator(sourceVertex->state());
    auto dubinsComp = getDubinsComparator(sourceVertex->state());
    // heapify first by Euclidean distance
    std::make_heap(m_Samples.begin(), m_Samples.end(), comp);
    // Use more heaps to sort by Dubins distance, skipping the samples which are farther away this time.
    // Making all the vertices adds some allocation overhead but it lets us cache the dubins paths
    std::vector<Vertex::SharedPtr> bestSamplesHeaps[nTurningRadii];
    bool doneChecks[nTurningRadii] = {false, false};
    // iterate through samples in closest (Euclidean distance) first order
    for (uint64_t i = 0; i < m_Samples.size() && (!doneChecks[0] || !doneChecks[1]); i++) {
        // get closest sample
        auto sample = m_Samples.front();
        std::pop_heap(m_Samples.begin(), m_Samples.end() - i, comp);
        // iterate through turning radii
        for (unsigned long j = 0; j < nTurningRadii; j++) {
            // if we've filled up the heap for this radius we can skip
            if (doneChecks[j]) continue;
            const auto& turningRadius = turningRadii[j];
            // if this radius isn't being used we can skip
            if (turningRadius <= 0) {
                doneChecks[j] = true;
                continue;
            }
            // grab the appropriate heap
            auto& bestSamples = bestSamplesHeaps[j];
            // if we haven't filled up the heap yet or this sample could possibly be better than the worst sample
            // we've connected to so far, add it to the heap
            if (bestSamples.size() < k() || bestSamples.front()->parentEdge()->getPlan(m_Config).length() >
                    sample.distanceTo(sourceVertex->state())){
                if (sourceVertex->state().distanceTo(sample) > m_Config.collisionCheckingIncrement()) {
                    // set the speed to be the max speed for now - it could get changed later
                    sample.speed() = m_Config.maxSpeed();
                    // check whether to allow coverage
                    bool coverageAllowed = turningRadius == m_Config.coverageTurningRadius();
                    // connect to the sample and push it onto the heap
                    bestSamples.push_back(Vertex::connect(sourceVertex, sample, turningRadius, coverageAllowed));
                    // make sure to compute the approx cost before fixing the heap
                    bestSamples.back()->parentEdge()->computeApproxCost();
                    // fix the heap
                    std::push_heap(bestSamples.begin(), bestSamples.end(), dubinsComp);
                    // if we've filled up the heap, pop the worst sample
                    if (bestSamples.size() > k()) {
                        std::pop_heap(bestSamples.begin(), bestSamples.end(), dubinsComp);
                        bestSamples.pop_back();
                    }
                }
            } else {
                // otherwise we're done with this turning radius (and heap)
                doneChecks[j] = true;
            }
        }
    }
    for (auto& bestSamples : bestSamplesHeaps) {
        // Push the closest K onto the open list
        if (bestSamples.size() > k()) throw std::runtime_error("Somehow got too many samples in the heap");
        for (auto& destinationVertex : bestSamples) {
            // use the wrapper from the vertex to save re-computing it but ditch the rest
            auto wrapper = destinationVertex->parentEdge()->getPlan(m_Config);
            for (const auto& speed : speeds) {
                if (speed <= 0) continue;
                // Changing the end state's speed will cause recalculation of approx cost if necessary
                wrapper.setSpeed(speed);
                auto v = Vertex::connect(sourceVertex, wrapper, destinationVertex->coverageAllowed());
                v->parentEdge()->computeTrueCost(m_Config);
                pushVertexQueue(v);
            }
        }
    }
    m_Stats.Expanded++;
}

int SamplingBasedPlanner::k() const {
    return m_Config.branchingFactor();
}

void SamplingBasedPlanner::addSamples(StateGenerator& generator, int n) {
    m_AttemptedSamples += n;
    for (int i = 0; i < n; i++) {
        const auto s = generator.generate();
        if (!m_Config.map()->isBlocked(s.x(), s.y()))
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

Planner::Stats SamplingBasedPlanner::plan(const RibbonManager&, const State& start, PlannerConfig config,
                                      const DubinsPlan& previousPlan,
                                      double timeRemaining) {
    m_Config = config;
    m_StartStateTime = start.time();
    m_Samples.clear();
    m_VertexQueue.clear();
    m_Stats = Stats();
    double minX, maxX, minY, maxY, minSpeed = m_Config.maxSpeed(), maxSpeed = m_Config.maxSpeed();
    double magnitude = m_Config.maxSpeed() * m_Config.timeHorizon();
    minX = start.x() - magnitude;
    maxX = start.x() + magnitude;
    minY = start.y() - magnitude;
    maxY = start.y() + magnitude;

    StateGenerator generator = StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, 7, m_RibbonManager); // lucky seed
    addSamples(generator, 1000);
    std::shared_ptr<Vertex> vertex;
    for (vertex = Vertex::makeRoot(start, m_RibbonManager);
         !goalCondition(vertex); vertex = popVertexQueue()) {
        expand(vertex, m_Config.obstaclesManager());
    }
    m_Stats.Plan = std::move(tracePlan(vertex, false, m_Config.obstaclesManager()));
    m_Stats.Samples = m_Samples.size();
    m_Stats.PlanDepth = vertex->getDepth();
    return m_Stats;
}

void SamplingBasedPlanner::visualizeVertex(Vertex::SharedPtr v, const std::string& tag, bool expanded) {
    if (m_Config.visualizations()) {
        m_Config.visualizationStream() << (expanded? "Expanded " : "Generated ") <<
        v->toString() << " " << tag << " " << v->getPointerTreeString() << std::endl;
    }
}

bool SamplingBasedPlanner::vertexQueueEmpty() const {
    return m_VertexQueue.empty();
}

void SamplingBasedPlanner::visualizeRibbons(const RibbonManager& ribbonManager) {
    if (m_Config.visualizations()) {
        m_Config.visualizationStream() << ribbonManager.dumpRibbons() << "\nEnd Ribbons" << std::endl;
    }
}

void SamplingBasedPlanner::visualizePlan(const DubinsPlan& plan) {
    if (m_Config.visualizations()) {
        State s;
        s.time() = plan.getStartTime();
        while (s.time() < plan.getEndTime()) {
            plan.sample(s);
            m_Config.visualizationStream() << "State: (" << s.toStringRad() << "), f: " << 0 << ", g: " << 0 << ", h: " <<
                                           0 << " plan" << std::endl;
            s.time() += 1;
        }
    }
}


