#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_TRAJECTORY_PUBLISHER_H

/**
 * Interface to expose trajectory publishing to the Executive
 */
class TrajectoryPublisher
{
public:
    virtual ~TrajectoryPublisher() = default;
    virtual void publishTrajectory(vector<State> trajectory) = 0;
    virtual void displayTrajectory(vector<State> trajectory, bool plannerTrajectory=true) = 0;
    virtual void allDone() = 0;
    virtual State getEstimatedState(double desiredTime) = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
