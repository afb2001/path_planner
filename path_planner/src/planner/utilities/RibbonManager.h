#ifndef SRC_RIBBONMANAGER_H
#define SRC_RIBBONMANAGER_H

#include <path_planner_common/State.h>
#include "Ribbon.h"

#include <list>
#include <vector>

extern "C" {
#include <dubins.h>
}

/**
 * Class that holds ribbons (survey lines).
 */
class RibbonManager {
public:
    /**
     * The different heuristics. None of them consider splitting survey lines at this time.
     */
    enum Heuristic {
        MaxDistance,
        TspPointRobotNoSplitAllRibbons,
        TspPointRobotNoSplitKRibbons,
        TspDubinsNoSplitAllRibbons,
        TspDubinsNoSplitKRibbons,
    };

    /**
     * Construct an empty ribbon manager.
     */
    RibbonManager();

    /**
     * Construct a ribbon manager with a specific heuristic.
     * @param heuristic
     */
    explicit RibbonManager(Heuristic heuristic);

    /**
     * Construct a ribbon manager with a specific turning radius.
     * @param heuristic
     * @param turningRadius
     */
    RibbonManager(Heuristic heuristic, double turningRadius);

    /**
     * Construct a ribbon manager with specific heuristic, turning radius and TSP-branch-limiting k
     * @param heuristic
     * @param turningRadius
     * @param k maximum branching factor for brute force TSP
     */
    RibbonManager(Heuristic heuristic, double turningRadius, int k);

    /**
     * Add a ribbon from (x1, y1) to (x2, y2)
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     */
    void add(double x1, double y1, double x2, double y2);

    /**
     * Update the ribbons by covering (x, y)
     * @param x
     * @param y
     */
    void cover(double x, double y, bool strict);

    /**
     * Update the ribbons by covering between (x1, y1) and (x2, y2)
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     */
    void coverBetween(double x1, double y1, double x2, double y2, bool strict);

    /**
     * @return whether the ribbons are all covered
     */
    bool done() const;

    /**
     * Calculate the heuristic value for the ribbons.
     * @param x
     * @param y
     * @param yaw
     * @return
     */
    double approximateDistanceUntilDone(double x, double y, double yaw) const;

    /**
     * If there are too many ribbons TSP solving is intractable so switch to max distance heuristic.
     */
    void changeHeuristicIfTooManyRibbons();

    /**
     * Find the smallest distance to a ribbon from (x, y).
     * @param x
     * @param y
     * @return
     */
    double minDistanceFrom(double x, double y) const;

    /**
     * Get the end point of a ribbon nearest to the given state, or the other end of a ribbon when the state is close to
     * one end
     * @param state
     * @return
     */
    State getNearestEndpointAsState(const State& state) const;

//    RibbonManager& operator=(const RibbonManager& other);

    /**
     * Get a string representation of the ribbons
     * @return
     */
    std::string dumpRibbons() const;

    /**
     * Project the state onto the nearest ribbon to it.
     * @param state
     */
    void projectOntoNearestRibbon(State& state) const;

    /**
     * Access the underlying list of ribbons
     * @return
     */
    const std::list<Ribbon>& get() const;

    /**
     * Find states on nearby ribbons radius distance away from the state.
     * Currently unused.
     * @param center
     * @param radius
     * @return
     */
    std::vector<State> findStatesOnRibbonsOnCircle(const State& center, double radius) const;

    /**
     * Compute "Brown paths" to nearby ribbons. Brown paths are minimal length Dubins curves that end on a line.
     * @param start
     * @param radius
     * @return
     */
    std::vector<State> findNearStatesOnRibbons(const State& start, double radius) const;

    /**
     * Change the heuristic.
     * @param heuristic
     */
    void setHeuristic(Heuristic heuristic);

    /**
     * Change the ribbon width.
     * @param lineWidth
     */
    static void setRibbonWidth(double lineWidth) { Ribbon::RibbonWidth = lineWidth; }

    /**
     * Get the time at which coverage was completed. If coverage has not yet been completed this will be < 0.
     * @return
     */
    double coverageCompletedTime() const;

    /**
     * Set the time at which coverage was completed. This should only be called once, at the time during collision
     * checking when coverage is completed.
     * @param coverageCompletedTime
     */
    void setCoverageCompletedTime(double coverageCompletedTime);

    /**
     * Get the total length of uncovered ribbons.
     * @return
     */
    double getTotalUncoveredLength() const;

private:
    // which heuristic to use
    Heuristic m_Heuristic;

    // turning radius for the Dubins TSP heuristics
    double m_TurningRadius = -1;

    // K for K nearest TSP heuristic variants
    int m_K;

    // record when coverage is done so we know when to stop afterwards
    double m_CoverageCompletedTime = -1;

    /**
     * List of (uncovered) ribbons.
     * This is a list (instead of, say, a vector) because there need to be constant time insertions and deletions. Plus,
     * it never needs to be accessed by index.
     */
    std::list<Ribbon> m_Ribbons;

    /**
     * Calculate the Dubins distance between (x, y, h) and the state s.
     * @param x
     * @param y
     * @param h
     * @param s
     * @return
     */
    double dubinsDistance(double x, double y, double h, const State& s) const {
        if (m_TurningRadius == -1) throw std::logic_error("Cannot compute ribbon dubins distance with unset turning radius");
        DubinsPath dubinsPath;
        double q1[] = {x, y, h}, q2[] = {s.x(), s.y(), s.yaw()};
        dubins_shortest_path(&dubinsPath, q1, q2, m_TurningRadius);
        return dubins_path_length(&dubinsPath);
    }

    void add(const Ribbon& r, std::list<Ribbon>::iterator i, bool strict);

    /**
     * Calculate the max distance heuristic.
     * @param x
     * @param y
     * @return
     */
    double maxDistance(double x, double y) const;

    /**
     * Calculate the TSPDubinsNoSplitAllRibbons heuristic. As the name suggests, this uses Dubins distance between
     * ribbons, does not split ribbons, and does not limit the branching factor in the TSP.
     * @param ribbonsLeft
     * @param distanceSoFar
     * @param x
     * @param y
     * @param yaw
     * @return
     */
    double tspDubinsNoSplitAllRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, double x,
                                      double y, double yaw) const;

    /**
     * Calculate the TSPPointRobotNoSplitKRibbons heuristic. As the name suggests, this uses Euclidean distance between
     * ribbons, does not split ribbons, and limits the TSP branching factor to K.
     * @param ribbonsLeft
     * @param distanceSoFar
     * @param point
     * @return
     */
    double tspPointRobotNoSplitKRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, std::pair<double, double> point) const;

    /**
     * Calculate the TSPDubinsNoSplitKRibbons heuristic. As the name suggests, this uses Dubins distance between
     * ribbons, does not split ribbons, and limits the TSP branching factor to K.
     * @param ribbonsLeft
     * @param distanceSoFar
     * @param x
     * @param y
     * @param yaw
     * @return
     */
    double tspDubinsNoSplitKRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, double x,
                                    double y, double yaw) const;

    /**
     * Threshold for the number of ribbons that is too many for TSP heuristics to reliably handle. This number is not
     * necessarily well-chosen as I haven't really benchmarked the TSP heuristics very much.
     */
    static constexpr int c_RibbonCountDangerThreshold = 5;


    static double distance(std::pair<double, double> p1, std::pair<double, double> p2) {
        return distance(p1.first, p1.second, p2.first, p2.second);
    }
    static double distance(std::pair<double, double> p, double x, double y){
        return distance(p.first, p.second, x, y);
    }
    /**
     * Euclidean distance between (x1, y1) and (x2, y2). Has a couple convenience overloads.
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    static double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    }

    /**
     * Calculate the TSPPointRobotNoSplitAllRibbons heuristic. As the name suggests, this uses Euclidean distance between
     * ribbons, does not split ribbons, and does not limit the TSP branching factor.
     * @param ribbonsLeft
     * @param distanceSoFar
     * @param point
     * @return
     */
    static double tspPointRobotNoSplitAllRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, std::pair<double, double> point);
};


#endif //SRC_RIBBONMANAGER_H
