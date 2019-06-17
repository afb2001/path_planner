#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_TRAJECTORY_PUBLISHER_H

/**
 * Interface to expose trajectory publishing to the Executive
 */
class TrajectoryPublisher
{
public:
    virtual ~TrajectoryPublisher() = default;
    /**
     * Publish a trajectory. This is pretty much the point of this interface.
     * @param trajectory the trajectory to publish
     */
    virtual void publishTrajectory(std::vector<State> trajectory) = 0;
    virtual void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory=true) = 0;
    /**
     * Alert the system that the planner has finished this iteration. This might deserve its own interface.
     */
    virtual void allDone() = 0;
    /**
     * Expose the state estimation service to the Executive. This might also deserve its own interface.
     * @param desiredTime the desired time for the estimated state
     * @return an estimated state at desiredTime
     */
    virtual State getEstimatedState(double desiredTime) = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
