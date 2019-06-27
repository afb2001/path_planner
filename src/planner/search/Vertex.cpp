#include "Vertex.h"

Vertex::Vertex(State state) {
    this->state = state;
}

Vertex::Vertex(State state, const std::shared_ptr<Edge>& parent) : Vertex(state) {
    this->parentEdge = parent;
}

std::shared_ptr<Vertex> Vertex::parent() const {
    return this->parentEdge->start();
}

bool Vertex::isRoot() const {
    if (parentEdge) return false;
    else return true;
}

//std::shared_ptr<Vertex> Vertex::connect(const std::weak_ptr<Vertex>& start, const State &next) {
//    auto e = new Edge(start);
//    return e->setEnd(next);
////    auto e = std::make_shared<Edge>(start, next);
////    return e->end();
//}

std::shared_ptr<Vertex> Vertex::connect(const std::shared_ptr<Vertex> &start, const State &next) {
    auto e = new Edge(start);
    return e->setEnd(next);
}

Vertex::~Vertex() = default;
