#ifndef SRC_EDGE_H
#define SRC_EDGE_H

#include "Vertex.h"
#include <path_planner_common/DubinsPlan.h>
#include <path_planner_common/DubinsWrapper.h>
#include "../PlannerConfig.h"

#include <memory>

extern "C" {
#include "dubins.h"
}

// forward declaration to resolve circular dependency
class Vertex;

/**
 * Class to represent an edge in graph search. This object knows about an starting and ending vertex. It computes a
 * Dubins path between their corresponding states, and then performs collision checking along it if necessary.
 * This class houses some important constants, too.
 *
 * There's some pretty high coupling going on between this class and the Vertex class, but that's because they're
 * kind of splitting responsibilities that really ought to be together in one thing. I wrote them separately in case
 * we wanted to implement BIT* someday, but I don't really see that happening. Anyway, for now, we're stuck with this
 * slightly weird architecture.
 *
 * See the Vertex header for resource management of edges and vertices.
 */
class Edge {
public:
    typedef std::shared_ptr<Edge> SharedPtr;

    /**
     * Construct an edge with a starting vertex. Don't call this manually - call Vertex::connect to expand the search tree.
     * @param start
     */
    Edge(std::shared_ptr<Vertex> start);

    ~Edge();

    /**
     * Retrieve the cached approximate cost (Dubins distance). Throws an exception if not cached.
     * @return
     */
    double approxCost() const;

    /**
     * Set the ending vertex of this edge with a state.
     * @param state
     * @return
     */
    std::shared_ptr<Vertex> setEnd(const State& state);

    /**
     * Set the ending vertex of this edge with a pre-computed Dubins curve. The path is sampled to determine the ending
     * state.
     * @param path
     * @return
     */
    std::shared_ptr<Vertex> setEnd(const DubinsWrapper& path);

    /**
     * Collision check the edge, computing the true cost. This also updates the ribbon manager associated with the
     * ending vertex.
     * @param config
     * @return
     */
    double computeTrueCost(PlannerConfig& config);

    /**
     * Retrieve the cached true cost, computing it if necessary.
     * @return
     */
    double trueCost() const;

    /**
     * Compute the Dubins curve between the states at the start and end vertices. That length, divided by the max speed,
     * is the approximate cost.
     * @param maxSpeed
     * @param turningRadius
     * @return
     */
    double computeApproxCost(double maxSpeed, double turningRadius);
    double computeApproxCost();

    /**
     * Fetch the Dubins path in this edge. Throws an exception if not computed yet.
     * @param config
     * @return
     */
    DubinsWrapper getPlan(const PlannerConfig& config);
    const DubinsWrapper& getPlan(const PlannerConfig& config) const { return m_DubinsWrapper; }

    /**
     * @return the start vertex.
     */
    std::shared_ptr<Vertex> start() const;

    /**
     * @return the end vertex.
     */
    std::shared_ptr<Vertex> end() const;

    /**
     * @return true iff the edge is infeasible (static obstacle)
     */
    bool infeasible() const;

    /**
     * @return the cached collision penalty. This is really just for debugging I think.
     */
    double getSavedCollisionPenalty() const { return m_CollisionPenalty; }

    // public constants
    /**
     * The collision penalty factor. Can be seen as a conversion from whatever the collision penalty units are to score
     * units, which the rest of the costs are represented in. Unless someone changed it, score units are in seconds, but
     * have a look at c_TimePenaltyFactor just in case (if it's 1, score is in seconds).
     * @return
     */
    static double collisionPenaltyFactor() { return c_CollisionPenaltyFactor; }

    /**
     * Time penalty factor. This can be seen as a conversion from seconds to score units.
     * @return
     */
    static double timePenaltyFactor() { return c_TimePenaltyFactor; }

private:
    // Edges own their parent vertices
    std::shared_ptr<Vertex> m_Start;
    // Vertices own their parent edges
    std::weak_ptr<Vertex> m_End;

    DubinsWrapper m_DubinsWrapper;

    bool m_Infeasible = false;

    double m_ApproxCost = -1, m_TrueCost = -1;

    double m_CollisionPenalty = 0;

    /**
     * Find the net time of the edge.
     * @return
     */
    double netTime();

    static constexpr double c_CollisionPenaltyFactor = 600; // no idea how to set this but this is probably too low (try 600)
    static constexpr double c_TimePenaltyFactor = 1;
};


#endif //SRC_EDGE_H
