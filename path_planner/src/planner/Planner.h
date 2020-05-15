#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H


#include <vector>
#include <path_planner_common/State.h>
#include "../common/map/Map.h"
#include "search/Vertex.h"
#include <path_planner_common/DubinsPlan.h>
#include "PlannerConfig.h"

/**
 * Interface to represent all planners. This might not have been really necessary but when I ported everything to C++
 * I started with implementing a dumb planner here and more complex up the hierarchy. The dumber planners didn't get
 * updated when we switched to ribbons so now this is basically just an interface.
 */
class Planner {
public:
    Planner();

    virtual ~Planner() = default;

    /**
     * Plan using the provided planning problem and configuration. Guaranteed to return before timeRemaining has elapsed.
     * @param ribbonManager the ribbon manager
     * @param start the start state
     * @param config planner configuration
     * @param previousPlan previous plan to help seed search
     * @param timeRemaining computation time bound
     * @return
     */
    virtual DubinsPlan plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
                            const DubinsPlan& previousPlan, double timeRemaining);

    /**
     * Construct a single plan by tracing back from the given vertex to the root.
     * @param v
     * @param smoothing
     * @param obstacles
     * @return
     */
    DubinsPlan tracePlan(const std::shared_ptr<Vertex>& v, bool smoothing, const DynamicObstaclesManager& obstacles);

    /**
     * Manually set the planner config. Meant for testing.
     * @param config
     */
    void setConfig(PlannerConfig config);

protected:

    /**
     * Utility to get the current time in seconds.
     * @return
     */
    double now() const;

    PlannerConfig m_Config;

};


#endif //SRC_PLANNER_H
