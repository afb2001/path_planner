#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_TRAJECTORY_PUBLISHER_H

#include "planner/utilities/RibbonManager.h"
#include <path_planner_common/DubinsPlan.h>

/**
 * Interface to expose trajectory publishing to the Executive
 * TODO! -- change name
 */
class TrajectoryPublisher
{
public:
    virtual ~TrajectoryPublisher() = default;
    /**
     * Publish a trajectory. This is pretty much the point of this interface.
     * @param trajectory the trajectory to publish
     * @return The state to plan from next, as predicted by a controller accepting the trajectory
     */
//    virtual State publishTrajectory(std::vector<State> trajectory) = 0;

    virtual State publishPlan(const DubinsPlan& plan) = 0;

    virtual void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory) = 0;
    /**
     * Alert the system that the planner has finished this iteration. This might deserve its own interface.
     */
    virtual void allDone() = 0;

    /**
     * Get the current time.
     * @return the current time in seconds
     */
    virtual double getTime() const = 0;

    virtual void displayRibbons(const RibbonManager& ribbonManager) = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
