#include <cfloat>
#include "Vertex.h"

Vertex::Vertex(State state) {
    this->m_State = state;
}

Vertex::Vertex(State state, const std::shared_ptr<Edge>& parent) : Vertex(state) {
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
    auto e = new Edge(start);
    return e->setEnd(next);
}

std::shared_ptr<Vertex> Vertex::makeRoot(const State& start, const Path& uncovered) {
    auto v = std::shared_ptr<Vertex>(new Vertex(start));
    v->m_CurrentCost = 0;
    v->m_Uncovered = uncovered;
    return v;
}

double Vertex::estimateApproxToGo(const State &destination) {
    return 0; // TODO
}

double Vertex::computeApproxToGo() {
    // NOTE: using the current speed for computing time penalty by distance. With just one speed it works.
    if (c_Heuristic == "tsp") {
        m_ApproxToGo = 0; // TODO! -- tsp solver
    } else if (c_Heuristic == "greedy") {
        auto nearest = getNearestPoint();
        m_ApproxToGo = state().distanceTo(nearest.first, nearest.second) / state().speed * Edge::timePenalty();
    } else if (c_Heuristic == "maxD") {
        double max = m_Uncovered.maxDistanceFrom(state());
        m_ApproxToGo = max / state().speed * Edge::timePenalty();
    } else {

    }
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

std::pair<double, double> Vertex::getNearestPoint() const {
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
    return nearest;
}

double Vertex::f() {
    return currentCost() + approxToGo();
}

void Vertex::setCurrentCost() {
    m_CurrentCost = parent()->currentCost() + parentEdge()->trueCost();
}

Vertex::~Vertex() = default;
