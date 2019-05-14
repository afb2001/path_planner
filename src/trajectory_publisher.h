#ifndef SRC_TRAJECTORY_PUBLISHER_H
#define SRC_TRAJECTORY_PUBLISHER_H

/**
 * Interface to expose trajectory publishing to the Executive
 */
class TrajectoryPublisher
{
public:
    virtual ~TrajectoryPublisher() = default;
    virtual void publishTrajectory(State* trajectory) = 0;
    virtual void allDone() = 0;
};


#endif //SRC_TRAJECTORY_PUBLISHER_H
