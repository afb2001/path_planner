#ifndef SRC_DUBINSWRAPPER_H
#define SRC_DUBINSWRAPPER_H

#include <path_planner_common/State.h>
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

    /**
     * Gets the start time. If the start time has been updated, that updated value is returned.
     * @return
     */
    double getStartTime() const;

    double getEndTime() const;

    void updateEndTime(double endTime);

    /**
     * Updates the start time. The original start time is kept for the math of sampling but for all other purposes this
     * is the start time now. It is not valid to sample before this time any more.
     * @param startTime
     */
    void updateStartTime(double startTime);

    const DubinsPath& unwrap() const;

private:
    DubinsPath m_DubinsPath;
    double m_Speed;
    double m_StartTime = -1, m_EndTime = -1, m_UpdatedStartTime = -1;

    bool isInitialized() const;

    void setEndTime();
};


#endif //SRC_DUBINSWRAPPER_H
