#ifndef SRC_SAMPLINGBASEDPLANNER_H
#define SRC_SAMPLINGBASEDPLANNER_H

#include "Planner.h"
#include "utilities/StateGenerator.h"
#include <functional>

/**
 * Class initially designed to represent a uniform cost search planner. That implementation didn't get updated when we
 * switched to ribbons, so now this just houses half the AStarPlanner code (expansion).
 */
class SamplingBasedPlanner : public Planner {
public:
    SamplingBasedPlanner();

    ~SamplingBasedPlanner() override = default;

    Stats plan(const RibbonManager&, const State& start, PlannerConfig config, const DubinsPlan& previousPlan,
                    double timeRemaining) override;

    /**
     * Push a vertex onto the open list.
     * @param vertex
     */
    void pushVertexQueue(Vertex::SharedPtr vertex);

    /**
     * Get and remove the front of the open list.
     * @return
     */
    std::shared_ptr<Vertex> popVertexQueue();

    /**
     * Clear the open list.
     */
    void clearVertexQueue();

    /**
     * Expand a vertex, pushing its children onto the open list.
     * @param sourceVertex
     * @param obstacles
     */
    virtual void expand(const std::shared_ptr<Vertex>& sourceVertex, const DynamicObstaclesManager& obstacles);

    /**
     * Increase the number of samples.
     * @param generator
     */
    void addSamples(StateGenerator& generator);
    void addSamples(StateGenerator& generator, int n);

protected:
    double m_StartStateTime;
    std::vector<State> m_Samples;
    int m_ExpandedCount = 0;

    Vertex::SharedPtr m_BestVertex;

    RibbonManager m_RibbonManager;

    /**
     * Retrieve a function that compares vertices. This orders the open list. Probably over-complicated but very general.
     * @return
     */
    virtual std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator();

    /**
     * Goal condition on which to stop search.
     * @param vertex
     * @return
     */
    bool goalCondition(const std::shared_ptr<Vertex>& vertex);

    /**
     * Branching factor for expansion.
     * @return
     */
    virtual int k() const;

    /**
     * Visualize a vertex with the given tag.
     * @param v
     * @param tag
     */
    void visualizeVertex(Vertex::SharedPtr v, const std::string& tag, bool expanded);

    void visualizePlan(const DubinsPlan& plan);

    /**
     * Visualize a ribbon manager.
     * @param ribbonManager
     */
    void visualizeRibbons(const RibbonManager& ribbonManager);

    /**
     * Check whether the open list is empty.
     * @return
     */
    bool vertexQueueEmpty() const;

private:
    std::vector<std::shared_ptr<Vertex>> m_VertexQueue;

    /**
     * State comparison to order samples for Dubins path computation.
     * @param origin
     * @return
     */
    std::function<bool(const State& s1, const State& s2)> getStateComparator(const State& origin);

    /**
     * Vertex comparison which uses Dubins distance to order expansion.
     * @param origin
     * @return
     */
    std::function<bool(const std::shared_ptr<Vertex>&, const std::shared_ptr<Vertex>&)> getDubinsComparator(
            const State& origin);
};


#endif //SRC_SAMPLINGBASEDPLANNER_H
