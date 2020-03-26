#include "DynamicObstacle.h"

double DynamicObstacle::distanceToEdge(double x, double y, double speed, double time) const {
    // TODO! -- implement
    return 0;
}

double DynamicObstacle::collisionDensityAt(double x, double y, double time) const {
    // find the highest time lower than the desired time with a binary search
    unsigned long lower = 0, upper = m_Distributions.size() - 1;
    auto i = upper / 2;
    while (lower + 1 < upper){
        if (m_Distributions[i].time() <= time) {
            lower = i;
        } else {
            upper = i;
        }
        i = (lower + upper) / 2;
    }
    auto interpolated = m_Distributions[lower].interpolate(m_Distributions[upper], time);
    // could this all be done any faster?
    // calculate the distance from the test point to the mean
    auto dx = x - interpolated.mean()[0];
    auto dy = y - interpolated.mean()[1];
    auto d = sqrt(dx*dx + dy*dy);
    // convert the distance to be in terms of our length and width (because we're rotated by our heading)
    auto beta = atan2(dy, dx);
    auto alpha = beta + interpolated.heading(); // I think this is right because we're using heading not yaw
    auto dl = d * sin(alpha);
    auto dw = d * cos(alpha);
    // check if the test point is inside our rectangle
    if (dl < m_Length / 2 && dw < m_Width / 2) {
        return interpolated.density(interpolated.mean()); // it is, so just return the density at the mean
    }
    // move us toward the mean of the distribution by an amount dictated by our size and orientation
    auto dInner = m_Length / (2 * sin(alpha));
    x -= dInner * cos(beta);
    y -= dInner * sin(beta);

    // return the density at the translated test point
    return interpolated.density(x, y);
}

DynamicObstacle::DynamicObstacle(const std::vector<Distribution>& distributions)
    : DynamicObstacle(distributions, c_DefaultLength, c_DefaultWidth) {}

DynamicObstacle::DynamicObstacle(const std::vector<Distribution>& distributions, double length, double width) {
    m_Length = length;
    m_Width = width;
    m_Distributions = distributions;
}

void DynamicObstacle::update(const std::vector<Distribution>& distributions) {
    m_Distributions = distributions;
}
