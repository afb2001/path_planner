#ifndef SRC_DUBINSWRAPPER_H
#define SRC_DUBINSWRAPPER_H

#include <path_planner_common/State.h>
#include <vector>

extern "C" {
#include "dubins.h"
};

/**
 * Wrapper class for convenience around a Dubins path. Check out the included "dubins.h" for more details about that.
 */
class DubinsWrapper {
public:
    DubinsWrapper() = default;

    /**
     * Construct a Dubins wrapper. Computes and stores the Dubins path between the given states.
     * @param s1 initial configuration
     * @param s2 final configuration
     * @param rho turning radius
     */
    DubinsWrapper(const State& s1, const State& s2, double rho);

    /**
     * Computes and stores the Dubins path between the given states.
     * @param s1 initial configuration
     * @param s2 final configuration
     * @param rho turning radius
     */
    void set(const State& s1, const State& s2, double rho);

    /**
     * Stores the already-computed Dubins path.
     * @param path the Dubins path
     * @param speed
     * @param startTime
     */
    void fill(const DubinsPath& path, double speed, double startTime);

    /**
     * @return the length of the underlying Dubins path
     */
    double length() const;

    /**
     * @return true iff the given time occurs in this path
     */
    bool containsTime(double) const;

    /**
     * Sets the values of the given state to be on this Dubins path. Uses the provided time to determine how far along
     * the path it is. Throws a runtime error if the time is not contained within this path. DOES set the state's speed.
     * @param s
     */
    void sample(State& s) const;

    /**
     * Get samples at a constant time interval, starting at the starting time for this path.
     * @param timeInterval
     * @return
     */
    std::vector<State> getSamples(double timeInterval, double offset) const;

    /**
     * @return the turning radius
     */
    double getRho() const;

    /**
     * @return the speed
     */
    double getSpeed() const;

    /**
     * Gets the start time. If the start time has been updated, that updated value is returned.
     * @return
     */
    double getStartTime() const;

    /**
     * @return the end time, calculated based on the length and speed, or updated manually
     */
    double getEndTime() const;

    /**
     * @return the total time (end - start)
     */
    double getNetTime() const;

    /**
     * Truncate the path with a new (earlier) end time. Throws an exception if you try to make it longer (later end time).
     * @param endTime
     */
    void updateEndTime(double endTime);

    /**
     * Updates the start time. The original start time is kept for the math of sampling but for all other purposes this
     * is the start time now. It is not valid to sample before this time any more.
     * @param startTime
     */
    void updateStartTime(double startTime);

    /**
     * Get the underlying Dubins path.
     * @return
     */
    const DubinsPath& unwrap() const;

private:
    DubinsPath m_DubinsPath{};
    double m_Speed{};
    double m_StartTime = -1, m_EndTime = -1, m_UpdatedStartTime = -1;

    /**
     * Because you can default-construct it. Would be nice to get rid of this.
     * @return
     */
    bool isInitialized() const;

    /**
     * Compute the end time based on length and speed.
     */
    void setEndTime();
};


#endif //SRC_DUBINSWRAPPER_H
