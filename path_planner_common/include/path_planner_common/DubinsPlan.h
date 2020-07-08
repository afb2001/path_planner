#ifndef SRC_DUBINSPLAN_H
#define SRC_DUBINSPLAN_H

#include <vector>
#include <path_planner_common/State.h>
#include "DubinsWrapper.h"

/**
 * Class to represent a plan made up of a sequence of Dubins paths. Can do some convenient things like sample states and
 * query the total time of the plan. Some important constants are housed here as well. Maybe they shouldn't be here but
 * this is pretty self-contained, and everything that uses them also uses this. Maybe they should be constructor parameters?
 *
 * The paths are meant to be continuous in time, and in order, but that is not currently enforced.
 */
class DubinsPlan {
public:
    /**
     * Construct an empty plan.
     */
    DubinsPlan() = default;

    /**
     * Construct a plan with one Dubins path from s1 to s2.
     * @param s1 initial state
     * @param s2 final state
     * @param rho turning radius
     */
    DubinsPlan(const State& s1, const State& s2, double rho);

    /**
     * Append a plan to this one. Does no checking for valid, ordered times, but probably should.
     * @param plan
     */
    void append(const DubinsPlan& plan);

    /**
     * Appends a Dubins path to the plan.
     * @param dubinsPath
     */
    void append(const DubinsWrapper& dubinsPath);

    /**
     * Samples a state along the plan. Uses the time set in the state. This sets the speed of the state, too.
     * @param s
     */
    void sample(State& s) const;

    /**
     * @return whether the plan is empty
     */
    bool empty() const;

    /**
     * @return the total time from the beginning of the first path to the end of the last.
     */
    double totalTime() const;

    /**
     * @return the start time of the first path.
     */
    double getStartTime() const;

    /**
     * @return the end time of the last path.
     */
    double getEndTime() const;

    /**
     * Determines whether the given time is contained in the paths in this plan.
     * @param time
     * @return
     */
    bool containsTime(double time) const;

    /**
     * Truncate this plan to start at the given time. Hey this doesn't actually do anything. Huh. Should look into that.
     * @param startTime
     */
    void changeIntoSuffix(double startTime);

    /**
     * Get samples at half second intervals.
     * @return
     */
    std::vector<State> getHalfSecondSamples() const;

    /**
     * Get the underlying container of Dubins wrappers.
     * @return
     */
    const std::vector<DubinsWrapper>& get() const;

    /**
     * The old density at which plans were sampled. I don't think this needs to still be here.
     * @return
     */
    static constexpr double planTimeDensity() { return c_PlanTimeDensity; }

private:
    std::vector<DubinsWrapper> m_DubinsPaths;

    bool m_Dangerous = false;
public:
    bool dangerous() const;

    void setDangerous(bool dangerous);

private:

    static constexpr double c_PlanTimeDensity = 0.5;
};


#endif //SRC_DUBINSPLAN_H
