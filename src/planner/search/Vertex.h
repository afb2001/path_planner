#ifndef SRC_VERTEX_H
#define SRC_VERTEX_H

#include <memory>
#include <path_planner/State.h>
#include "Edge.h"
#include "../common/Path.h"

class Edge;
class Vertex {
public:
    State state;
    std::shared_ptr<Edge> parentEdge; // vertex owns its parent edge
    Path uncovered;
    double currentCost;
    bool currentCostIsSet;

    /**
     * Construct the root vertex (no parent edge).
     * @param state the starting state
     */
    Vertex(State state);

    /**
     * Construct a non-root vertex. Not for external use.
     * @param state the underlying state
     * @param parent the parent edge
     */
    Vertex(State state, const std::shared_ptr<Edge>& parent);

    /**
     * Create a new vertex with underlying state @next, and an edge between @start and the new vertex.
     * Use this instead of any constructors for non-root vertices.
     * @param start the starting vertex
     * @param next the ending state
     * @return a vertex connected by a new edge to @start
     */
//    static std::shared_ptr<Vertex> connect(const std::weak_ptr<Vertex>& start, const State& next);
    static std::shared_ptr<Vertex> connect(const std::shared_ptr<Vertex>& start, const State& next);

    ~Vertex();

    std::shared_ptr<Vertex> parent() const;

    bool isRoot() const;
private:
    double m_ApproxCost;
    double m_ApproxToGo;
};


#endif //SRC_VERTEX_H
