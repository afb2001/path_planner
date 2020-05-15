#include <cfloat>
#include <sstream>
#include "Vertex.h"

Vertex::Vertex(State state, bool useRibbons) : m_UseRibbons(useRibbons) {
    this->m_State = state;
}

Vertex::Vertex(State state, const std::shared_ptr<Edge>& parent, bool useRibbons) : Vertex(state, useRibbons) {
    this->m_ParentEdge = parent;
}

const std::string Vertex::c_Heuristic = "maxD";

std::shared_ptr<Vertex> Vertex::parent() const {
    return this->m_ParentEdge->start();
}

bool Vertex::isRoot() const {
    if (m_ParentEdge) return false;
    else return true;
}

std::shared_ptr<Vertex> Vertex::connect(const std::shared_ptr<Vertex> &start, const State &next) {
    auto e = new Edge(start, start->m_UseRibbons);
    auto v = e->setEnd(next);
    if (start->m_UseRibbons) {
        v->m_RibbonManager = start->m_RibbonManager;
    }
    return v;
}

Vertex::SharedPtr Vertex::connect(const Vertex::SharedPtr& start, const DubinsWrapper& wrapper) {
    auto e = new Edge(start, start->m_UseRibbons);
    auto v = e->setEnd(wrapper);
    if (start->m_UseRibbons) {
        v->m_RibbonManager = start->m_RibbonManager;
    }
    return v;
}

std::shared_ptr<Vertex> Vertex::makeRoot(const State& start, const Path& uncovered) {
    auto v = std::shared_ptr<Vertex>(new Vertex(start, false));
    v->m_CurrentCost = 0;
    v->m_Uncovered = uncovered;
    return v;
}

Vertex::SharedPtr Vertex::makeRoot(const State& start, const RibbonManager& ribbons) {
    auto v = std::shared_ptr<Vertex>(new Vertex(start, true));
    v->m_CurrentCost = 0;
    v->m_RibbonManager = ribbons;
    return v;
}

double Vertex::estimateApproxToGo(const State &destination) {
    return 0; // TODO
}

double Vertex::computeApproxToGo() {
    // NOTE: using the current speed for computing time penalty by distance. With just one speed it works.
    // TODO -- pass planner config to retrieve max speed instead of this assumption
    double max;
    if (m_UseRibbons) max = m_RibbonManager.approximateDistanceUntilDone(state().x(), state().y(), state().heading());
    else max = m_Uncovered.maxDistanceFrom(state());
    m_ApproxToGo = max / state().speed() * Edge::timePenalty();

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

Path& Vertex::uncovered() {
    return m_Uncovered;
}

double Vertex::currentCost() const {
    return m_CurrentCost;
}

double Vertex::approxToGo() {
    if (m_ApproxToGo == -1) computeApproxToGo();
    return m_ApproxToGo;
}

int Vertex::getDepth() const {
    if (isRoot()) return 0;
    return 1 + parent()->getDepth();
}

State Vertex::getNearestPointAsState() const {
    if (m_UseRibbons) {
        if (m_RibbonManager.done()) throw std::logic_error("Getting nearest point with empty path");
        return m_RibbonManager.getNearestEndpointAsState(state());
    }
    if (m_Uncovered.size() == 0) throw std::logic_error("Getting nearest point with empty path");
    std::pair<double, double> nearest;
    auto minDistance = DBL_MAX;
    for (auto p : m_Uncovered.get()) {
        auto d = state().distanceTo(p.first, p.second);
        if (d < minDistance) {
            nearest = p;
            minDistance = d;
        }
    }
    return State(nearest.first, nearest.second, 0, 0, 0);
}

double Vertex::f() {
    assert(std::isfinite(currentCost() + approxToGo()));
    return currentCost() + approxToGo();
}

void Vertex::setCurrentCost() {
    m_CurrentCost = parent()->currentCost() + parentEdge()->trueCost();
}

bool Vertex::allCovered() const {
    if (m_UseRibbons) return m_RibbonManager.done();
    else return m_Uncovered.size() == 0;
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

Vertex::~Vertex() = default;
