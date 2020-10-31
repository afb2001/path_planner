#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H

#include <path_planner_common/State.h>
#include "../common/map/Map.h"
#include "search/Vertex.h"
#include <path_planner_common/DubinsPlan.h>
#include "PlannerConfig.h"
#include "utilities/RibbonManager.h"

#include <vector>

/**
 * Interface to represent all planners. This might not have been really necessary but when I ported everything to C++
 * I started with implementing a dumb planner here and more complex up the hierarchy. The dumber planners didn't get
 * updated when we switched to ribbons so now this is basically just an interface.
 */
class Planner {
public:
    /**
     * Hold all the stats for the planner.
     *
     * TODO! -- CPU time?
     */
    struct Stats {
        unsigned long Samples;
        unsigned long Generated;
        unsigned long Expanded;
        unsigned long Iterations;
        double PlanFValue;
        double PlanCollisionPenalty = 0;
        double PlanTimePenalty;
        double PlanHValue;
        unsigned long PlanDepth;
        DubinsPlan Plan;
    };

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
    virtual Stats plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
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

    Stats m_Stats;

};


#endif //SRC_PLANNER_H
