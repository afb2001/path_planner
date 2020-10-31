#ifndef SRC_VERTEX_H
#define SRC_VERTEX_H

#include <path_planner_common/State.h>
#include "../utilities/RibbonManager.h"
#include "path_planner_common/DubinsWrapper.h"
#include "../PlannerConfig.h"

#include <memory>
#include <string>

// forward declaration to resolve circular dependency
class Edge;

/**
 * Class to represent a vertex for graph search. The vertex holds a State and a ribbon manager, as well as some costs.
 * Because of the circular dependency with Edges, Vertices must be constructed with the provided static helper functions.
 * They ensure everything is hooked up correctly.
 *
 * This vertex and edge separation seems superfluous for A* search, but I wrote it this way to be easily upgraded to a
 * variant of BIT* if we deem that necessary.
 *
 * Pointer ownership structure:
 * A vertex owns the pointer to its parent edge. The root vertex owns nothing. Edges own pointers to their parent vertex
 * but hold only a weak pointer to their child vertex.
 */
class Vertex {
public:

    typedef std::shared_ptr<Vertex> SharedPtr;

    /**
     * Construct the root vertex (no parent edge).
     * Use Vertex::makeRoot for full root construction.
     * @param state the starting state
     */
    Vertex(State state);

    /**
     * Construct a non-root vertex. Use Vertex::connect for full non-root construction.
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
    static std::shared_ptr<Vertex> connect(const std::shared_ptr<Vertex>& start, const State& next);
    static std::shared_ptr<Vertex> connect(const std::shared_ptr<Vertex>& start, const State& next, double turningRadius, bool coverageAllowed);
    static Vertex::SharedPtr connect(const Vertex::SharedPtr& start, const DubinsWrapper& wrapper,
                                     bool coverageAllowed);

    /**
     * Construct a root vertex.
     * @param start
     * @param ribbons
     * @return
     */
    static Vertex::SharedPtr makeRoot(const State& start, const RibbonManager& ribbons);

    ~Vertex();

    /**
     * Get the parent of this vertex.
     * @return
     */
    std::shared_ptr<Vertex> parent() const;

    /**
     * @return true iff this is the root.
     */
    bool isRoot() const;

    /**
     * Estimate the heuristic value at a given state, assuming we don't cover anything along this edge to it.
     * Why would this be useful? Pretty much just for BIT*, I think. Maybe not even then.
     * @param destination the destination state
     * @return an estimate of the heuristic value at that state (from here)
     */
    double estimateApproxToGo(const State& destination);

    /**
     * Compute the heuristic value of this vertex. The heuristic type to use is set in the ribbon manager for some reason.
     * This value comes back as a time, assuming maximum speed.
     * @return
     */
    double computeApproxToGo(const PlannerConfig& config);

    /**
     * Get the underlying state from this vertex.
     * @return
     */
    State& state();
    const State& state() const;

    /**
     * Get the parent edge to this vertex.
     * @return
     */
    const std::shared_ptr<Edge>& parentEdge() const;

    /**
     * Retrieve the current cost (g).
     * @return
     */
    double currentCost() const;

    /**
     * Calculate the current cost (g).
     */
    void setCurrentCost();

    /**
     * Retrieve the approx cost to go (h). Calculates if not cached.
     * @return
     */
    double approxToGo();

    /**
     * Retrieve the f value (g + h). Calculates components if they aren't cached.
     * @return
     */
    double f();

    /**
     * Function which computes depth in the search tree. Linear in tree depth - this value is not cached. It isn't really
     * used any more, as it was just for the UCS planner.
     * @return
     */
    int getDepth() const;

    /**
     * Get the endpoint of the nearest ribbon as a state. The ribbon manager should set the heading appropriately.
     * @return
     */
    State getNearestPointAsState() const;

    /**
     * Check if we're done covering ribbons.
     * @return
     */
    bool done() const;

    /**
     * Grab a reference to the ribbon manager.
     * @return
     */
    RibbonManager& ribbonManager();
    const RibbonManager& ribbonManager() const;

    /**
     * Get a string representation of the vertex.
     * @return
     */
    std::string toString() const;

    /**
     * Get the pointer values for the motion tree to this vertex concatenated into a string.
     * @return
     */
    std::string getPointerTreeString() const;

    /**
     * Retrieve the turning radius.
     * @return
     */
    double turningRadius() const;

    /**
     * @return whether coverage is allowed *to* this vertex.
     */
    bool coverageAllowed() const;

private:

    State m_State;
    std::shared_ptr<Edge> m_ParentEdge; // vertex owns its parent edge
    RibbonManager m_RibbonManager;
    double m_CurrentCost = -1;
    double m_ApproxCost = -1; // I think this was for BIT* but it's not used right now.
    double m_ApproxToGo = -1;
    double m_TurningRadius;
    bool m_CoverageIsAllowed = false;
};


#endif //SRC_VERTEX_H
