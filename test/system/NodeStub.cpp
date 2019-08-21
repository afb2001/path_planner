#include "NodeStub.h"

using std::cerr;
using std::endl;

void NodeStub::publishTrajectory(std::vector<State> trajectory) {
    m_LastTrajectory = std::move(trajectory);
//    cerr << "NodeStub published trajectory: \n";
//    for (auto s : trajectory) cerr << s.toString() << endl;
//    cerr << endl;
}

void NodeStub::displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory) {
//    cerr << "NodeStub displayed" << (plannerTrajectory? " planner " : " controller ") <<  "trajectory: \n";
//    for (auto s : trajectory) cerr << s.toString() << endl;
//    cerr << endl;
}

void NodeStub::allDone() {
    m_AllDoneCalled = true;
    cerr << "NodeStub allDone called" << endl;
}

State NodeStub::getEstimatedState(double desiredTime) {
    cerr << "NodeStub called to estimate state at time " << desiredTime << endl;
    return State(-1);
}

std::vector<State> NodeStub::lastTrajectory() const {
    return m_LastTrajectory;
}

bool NodeStub::allDoneCalled() const {
    return m_AllDoneCalled;
}
