#ifndef SRC_VERTEX_H
#define SRC_VERTEX_H

#include <memory>
#include <path_planner/State.h>
#include "Edge.h"
#include "../utilities/Path.h"
#include "../utilities/RibbonManager.h"

class Edge;
class Vertex {
public:

    typedef std::shared_ptr<Vertex> SharedPtr;

    /**
     * Construct the root vertex (no parent edge).
     * Use Vertex::makeRoot for full root construction.
     * @param state the starting state
     * @param useRibbons whether to use the ribbon representation (decided by parameters to makeRoot)
     */
    Vertex(State state, bool useRibbons);

    /**
     * Construct a non-root vertex. Use Vertex::connect for full non-root construction.
     * @param state the underlying state
     * @param parent the parent edge
     * @param useRibbons whether to use the ribbon representation (inherited from parent)
     */
    Vertex(State state, const std::shared_ptr<Edge>& parent, bool useRibbons);

    /**
     * Create a new vertex with underlying state @next, and an edge between @start and the new vertex.
     * Use this instead of any constructors for non-root vertices.
     * @param start the starting vertex
     * @param next the ending state
     * @return a vertex connected by a new edge to @start
     */
    static std::shared_ptr<Vertex> connect(const std::shared_ptr<Vertex>& start, const State& next);

    static std::shared_ptr<Vertex> makeRoot(const State& start, const Path& uncovered);

    static Vertex::SharedPtr makeRoot(const State& start, const RibbonManager& ribbons);

    ~Vertex();

    std::shared_ptr<Vertex> parent() const;

    bool isRoot() const;

    /**
     * Estimate the heuristic value at a given state, assuming we don't cover anything along this edge to it.
     * Why would this be useful? We'll see I guess
     * @param destination the destination state
     * @return an estimate of the heuristic value at that state (from here)
     */
    double estimateApproxToGo(const State& destination);

    double computeApproxToGo();

    State& state();
    const State& state() const;

    const std::shared_ptr<Edge>& parentEdge() const;

    Path& uncovered();

    double currentCost() const;

    void setCurrentCost();

    double approxToGo();

    double f();

    int getDepth() const;

    State getNearestPointAsState() const;

    bool allCovered() const;

    RibbonManager& ribbonManager();

    std::string toString() const;
private:
    static const std::string c_Heuristic; // = "maxD";

    State m_State;
    std::shared_ptr<Edge> m_ParentEdge; // vertex owns its parent edge
    Path m_Uncovered;
    RibbonManager m_RibbonManager;
    double m_CurrentCost = -1;
    double m_ApproxCost = -1;
    double m_ApproxToGo = -1;
    bool m_UseRibbons;
};


#endif //SRC_VERTEX_H
