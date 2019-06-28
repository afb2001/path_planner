#include "Vertex.h"

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
    return e->setEnd(next);
}

double Vertex::estimateApproxToGo(const State &destination) {
    return 0; // TODO
}

double Vertex::computeApproxToGo() {
    if (HEURISTIC == "tsp") {
        return 0; // TODO! -- tsp solver
    } else {

    }
}

State& Vertex::state() {
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

void Vertex::setState(const State& state) {
    m_State = state;
}

void Vertex::setParentEdge(const std::shared_ptr<Edge>& parentEdge) {
    m_ParentEdge = parentEdge;
}

void Vertex::setUncovered(const Path& uncovered) {
    m_Uncovered = uncovered;
}

double Vertex::approxToGo() {
    if (m_ApproxToGo == -1) computeApproxToGo();
    return m_ApproxToGo;
}

Vertex::~Vertex() = default;
