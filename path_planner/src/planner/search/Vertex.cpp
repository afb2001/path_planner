#include <sstream>
#include "Vertex.h"
#include "Edge.h"

Vertex::Vertex(State state) {
    this->m_State = state;
}

Vertex::Vertex(State state, const std::shared_ptr<Edge>& parent) : Vertex(state) {
    this->m_ParentEdge = parent;
}

std::shared_ptr<Vertex> Vertex::parent() const {
    return this->m_ParentEdge->start();
}

bool Vertex::isRoot() const {
    if (m_ParentEdge) return false;
    else return true;
}

std::shared_ptr<Vertex> Vertex::connect(const std::shared_ptr<Vertex> &start, const State &next) {
    auto e = new Edge(start);
    auto v = e->setEnd(next);
    v->m_RibbonManager = start->m_RibbonManager;
    return v;
}

Vertex::SharedPtr Vertex::connect(const Vertex::SharedPtr& start, const DubinsWrapper& wrapper,
                                  bool coverageAllowed) {
    auto e = new Edge(start);
    auto v = e->setEnd(wrapper);
    v->m_RibbonManager = start->m_RibbonManager;
    v->m_CoverageIsAllowed = coverageAllowed;
    v->m_TurningRadius = wrapper.getRho();
    return v;
}

Vertex::SharedPtr Vertex::makeRoot(const State& start, const RibbonManager& ribbons) {
    auto v = std::shared_ptr<Vertex>(new Vertex(start));
    v->m_CurrentCost = 0;
    v->m_RibbonManager = ribbons;
    return v;
}

double Vertex::estimateApproxToGo(const State &destination) {
    return 0; // TODO - implement if you want to use BIT*
}

double Vertex::computeApproxToGo(const PlannerConfig& config) {
    double max;
    max = m_RibbonManager.approximateDistanceUntilDone(state().x(), state().y(), state().heading());
    // use max speed because we need a lower bound - we could go at max speed the rest of the way
    m_ApproxToGo = max / config.maxSpeed() * Edge::timePenaltyFactor();

    // since we're using the ribbon manager, we should have computed true cost at this point.
    // PathMax or whatever. Guarantees heuristic consistency which is required for f pruning
//    if (!isRoot()) {
//        auto parentF = parent()->f();
//        auto diff = parentF - f();
//        if (diff > 0) m_ApproxToGo += diff;
//    }

    return m_ApproxToGo;
}

State& Vertex::state() {
    return m_State;
}

const State& Vertex::state() const {
    return m_State;
}

const std::shared_ptr<Edge>& Vertex::parentEdge() const {
    return m_ParentEdge;
}

double Vertex::currentCost() const {
    return m_CurrentCost;
}

double Vertex::approxToGo() {
    if (m_ApproxToGo == -1) throw std::runtime_error("Fetching unset approx to go (h)");
    return m_ApproxToGo;
}

int Vertex::getDepth() const {
    if (isRoot()) return 0;
    return 1 + parent()->getDepth();
}

State Vertex::getNearestPointAsState() const {
    if (m_RibbonManager.done()) throw std::logic_error("Getting nearest point with empty path");
    return m_RibbonManager.getNearestEndpointAsState(state());
}

double Vertex::f() {
    assert(std::isfinite(currentCost() + approxToGo()));
    return currentCost() + approxToGo();
}

void Vertex::setCurrentCost() {
    m_CurrentCost = parent()->currentCost() + parentEdge()->trueCost();
}

bool Vertex::done() const {
    return m_RibbonManager.done();
}

RibbonManager& Vertex::ribbonManager() {
    return m_RibbonManager;
}

std::string Vertex::toString() const {
    std::stringstream stream;
    stream << "State: (" << state().toStringRad() << "), f: " << m_CurrentCost + m_ApproxToGo << ", g: " << m_CurrentCost
        << ", h: " << m_ApproxToGo;
    return stream.str();
}

double Vertex::turningRadius() const {
    return m_TurningRadius;
}

std::shared_ptr<Vertex> Vertex::connect(const std::shared_ptr<Vertex>& start, const State& next, double turningRadius, bool coverageAllowed) {
    auto v = connect(start, next);
    v->m_TurningRadius = turningRadius;
    v->m_CoverageIsAllowed = coverageAllowed;
    return v;
}

bool Vertex::coverageAllowed() const {
    return m_CoverageIsAllowed;
}

const RibbonManager& Vertex::ribbonManager() const {
    return m_RibbonManager;
}

std::string Vertex::getPointerTreeString() const {
    if (isRoot()) return std::to_string(reinterpret_cast<long>(this)) + " ";
    return parent()->getPointerTreeString() + std::to_string(reinterpret_cast<long>(this)) + " ";
}

Vertex::~Vertex() = default;
