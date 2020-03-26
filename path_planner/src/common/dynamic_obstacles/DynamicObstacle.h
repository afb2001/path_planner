#ifndef SRC_DYNAMICOBSTACLE_H
#define SRC_DYNAMICOBSTACLE_H

#include <path_planner_common/State.h>
#include "Distribution.h"
#include <vector>

/**
 * Models information about a dynamic obstacle. Specifically it holds a time series of distributions describing the
 * obstacle's position and heading.
 */
class DynamicObstacle {
public:
    /**
     * Construct a dynamic obstacle with default width and length.
     * @param distributions
     */
    explicit DynamicObstacle(const std::vector<Distribution>& distributions);

    /**
     * Construct a dynamic obstacle.
     * @param distributions
     * @param length
     * @param width
     */
    DynamicObstacle(const std::vector<Distribution>& distributions, double length, double width);

    /**
     * Update the distributions describing this dynamic obstacle.
     * @param distributions
     */
    void update(const std::vector<Distribution>& distributions);

    /**
     * Find the shortest distance from a point to the edge of the distribution at the specified time.
     * @param x
     * @param y
     * @param speed
     * @param time
     * @return
     */
    double distanceToEdge(double x, double y, double speed, double time) const;

    /**
     * Compute and return a number representing the density of collision with this obstacle at a particular time.
     * This number is not a probability but behaves a lot like a Gaussian probability density function.
     * @param x
     * @param y
     * @param time
     * @return
     */
    double collisionDensityAt(double x, double y, double time) const;

private:
    std::vector<Distribution> m_Distributions;
    double m_Length, m_Width;

    static constexpr double c_DefaultWidth = 3, c_DefaultLength = 3;

};


#endif //SRC_DYNAMICOBSTACLE_H
