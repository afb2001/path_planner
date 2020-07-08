#ifndef SRC_ASTARPLANNER_H
#define SRC_ASTARPLANNER_H

#include "SamplingBasedPlanner.h"

/**
 * The real deal planner, doing my real-time planning algorithm.
 */
class AStarPlanner : public SamplingBasedPlanner {
public:

    /**
     * Construct an AStarPlanner.
     */
    AStarPlanner() = default;

    ~AStarPlanner() override = default;

    Stats plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
                    const DubinsPlan& previousPlan, double timeRemaining) override;

protected:
    int m_IterationCount = 0;

    std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator() override;

    /**
     * Perform A* search using the open list, vertex queue, start state, etc.
     * @param obstacles
     * @param endTime
     * @return
     */
    std::shared_ptr<Vertex> aStar(const DynamicObstaclesManager& obstacles, double endTime);

    /**
     * Specifically expand root to connect to the given samples.
     * @param root
     * @param samples
     * @param obstacles
     * @param coverageAllowed
     */
    void expandToCoverSpecificSamples(Vertex::SharedPtr root, const std::vector<State>& samples,
                                      const DynamicObstaclesManager& obstacles, bool coverageAllowed);

};


#endif //SRC_ASTARPLANNER_H
