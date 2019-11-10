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
    State s(-1);
    // crudely estimate the state
    for (auto it = m_LastTrajectory.begin(); it != m_LastTrajectory.end(); it++) {
        if (it->time == desiredTime) return *it;
        if (it->time > desiredTime) {
            if (it == m_LastTrajectory.begin()) {
                s = *it;
            } else {
                s = *(it - 1);
            }
            break;
        }
    }
    if (!m_LastTrajectory.empty()) {
        if (s.time == -1) s = m_LastTrajectory.back();
        s.setEstimate(desiredTime - s.time, s);
    }
    return s;
}

std::vector<State> NodeStub::lastTrajectory() const {
    return m_LastTrajectory;
}

bool NodeStub::allDoneCalled() const {
    return m_AllDoneCalled;
}

double NodeStub::getTime() const {
    struct timespec t{};
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void NodeStub::displayRibbons(const RibbonManager& ribbonManager) {
    cerr << ribbonManager.dumpRibbons() << endl;
}
