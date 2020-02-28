#ifndef SRC_DUBINSWRAPPER_H
#define SRC_DUBINSWRAPPER_H

#include <path_planner/State.h>
#include <vector>

extern "C" {
#include "dubins.h"
};


class DubinsWrapper {
public:
    DubinsWrapper() = default;

    DubinsWrapper(const State& s1, const State& s2, double rho);

    void set(const State& s1, const State& s2, double rho);

    void fill(const DubinsPath& path, double speed, double startTime);

    double length() const;

    bool containsTime(double) const;

    void sample(State& s) const;

    std::vector<State> getSamples(double timeInterval) const;

    double getRho() const;

    double getSpeed() const;

    double getStartTime() const;

    double getEndTime() const;

    const DubinsPath& unwrap() const;

private:
    DubinsPath m_DubinsPath;
    double m_Speed;
    double m_StartTime = -1, m_EndTime = -1;

    bool isInitialized() const;

    void setEndTime();
};


#endif //SRC_DUBINSWRAPPER_H
