#ifndef SRC_NODESTUB_H
#define SRC_NODESTUB_H


#include <vector>
#include <path_planner/State.h>
#include "../../src/trajectory_publisher.h"
#include "../../src/planner/utilities/RibbonManager.h"

class NodeStub : public TrajectoryPublisher {
public:
    ~NodeStub() override = default;

    void publishTrajectory(std::vector<State> trajectory) override;

    void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory) override;

    void allDone() override;

    State getEstimatedState(double desiredTime) override;

    std::vector<State> lastTrajectory() const;
    
    bool allDoneCalled() const;

    double getTime() const override;

    void displayRibbons(const RibbonManager& ribbonManager) override;

private:
    std::vector<State> m_LastTrajectory;
    bool m_AllDoneCalled = false;
};


#endif //SRC_NODESTUB_H
