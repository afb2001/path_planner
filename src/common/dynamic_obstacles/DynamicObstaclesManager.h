#ifndef SRC_DYNAMICOBSTACLESMANAGER_H
#define SRC_DYNAMICOBSTACLESMANAGER_H


#include <path_planner/State.h>
#include "DynamicObstacle.h"
#include <unordered_map>

/**
 * Manages the dynamic obstacles for the executive.
 */
class DynamicObstaclesManager {
public:
    /**
     * Return a number weighted by increasing chance of collision. Not a probability, necessarily. Good luck tuning this.
     * @param s
     * @return not a probability
     */
    double collisionExists(const State& s) const;
    double collisionExists(double x, double y, double time) const;

    /**
     * Find the distance to the edge of the nearest truncated distribution representing a dynamic obstacle. Check to
     * make sure it's implemented before you use it.
     * @param s
     * @return distance (m)
     */
    double distanceToNearestPossibleCollision(const State& s) const;
    double distanceToNearestPossibleCollision(double x, double y, double speed, double time) const;

    /**
     * Add a new dynamic obstacle to be managed.
     * @param mmsi ID number for the obstacle
     * @param distributions
     * @param width
     * @param length
     */
    void add(uint32_t mmsi, const std::vector<Distribution>& distributions, double width, double length);

    /**
     * Update an existing dynamic obstacle. Does nothing if the obstacle is not already being managed.
     * @param mmsi
     * @param distributions
     */
    void update(uint32_t mmsi, const std::vector<Distribution>& distributions);

    /**
     * Forget a dynamic obstacle. Presumably it is no longer being tracked.
     * @param mmsi
     */
    void forget(uint32_t mmsi);

    /**
     * Ignore a specific dynamic obstacle. This is intended to be used to ignore a parent vessel to the ASV.
     * @param mmsi
     */
    void addIgnore(uint32_t mmsi);

    /**
     * Stop ignoring a specific dynamic obstacle.
     * @param mmsi
     */
    void removeIgnore(uint32_t mmsi);

private:
    std::unordered_map<uint32_t, DynamicObstacle> m_Obstacles;
    std::vector<uint32_t> m_IgnoreList;
};


#endif //SRC_DYNAMICOBSTACLESMANAGER_H
