#ifndef SRC_DYNAMICOBSTACLE_H
#define SRC_DYNAMICOBSTACLE_H

#include <path_planner/State.h>
#include "Distribution.h"
#include <vector>

class DynamicObstacle {
public:
    explicit DynamicObstacle(const std::vector<Distribution>& distributions);
    DynamicObstacle(const std::vector<Distribution>& distributions, double length, double width);

    void update(const std::vector<Distribution>& distributions);

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
