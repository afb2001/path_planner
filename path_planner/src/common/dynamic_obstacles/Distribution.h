#ifndef SRC_DISTRIBUTION_H
#define SRC_DISTRIBUTION_H

#include <cmath>
#include <cassert>

/**
 * Class that holds the representation of a truncated Gaussian distribution. This was written to model dynamic
 * obstacles for the path planner.
 */
class Distribution {
public:
    Distribution(const double (&mean)[2], const double (&covariance)[2][2], double width, double length, double heading, double time);

    virtual ~Distribution() = default;

    /**
     * Linearly interpolate between (or extrapolate with) this distribution and next, to desiredTime.
     * @param next the other distribution to interpolate with
     * @param desiredTime the desired time stamp of the interpolation
     * @return a new interpolated distribution
     */
    virtual Distribution interpolate(const Distribution& next, double desiredTime) const;

    /**
     * Interpolate between this distribution and next, to desiredTime, and then use the private density function to
     * compute the probability density at x.
     * @param next the other distribution to interpolate with
     * @param desiredTime the desired time of the interpolation
     * @param x the test point
     * @return probability density at x
     */
    virtual double density(const Distribution& next, double desiredTime, const double (&x)[2]) const;
    // convenience overload; no need to override as it calls the virtual one
    double density(const Distribution& next, double desiredTime, double x, double y) const;

    /**
     * Just use the private density function to calculate the probability density at x.
     * @param x the test point
     * @return the probability density at x
     */
    double density(const double (&x)[2]) const;
    // convenience overload
    double density(double x, double y) const;

    double heading() const;

    double time() const;

    const double (&mean() const) [2];

private:
    // by convention, order the means (and covariance) x, y
    double m_Mean[2];
    double m_Covariance[2][2];
    double m_Width, m_Length;
    double m_Time, m_Heading;


    /**
     * Fast inline computation of Gaussian density function in two dimensions at the requested x.
     * @param mean mean of the distribution
     * @param covariance covariance matrix of the distribution
     * @param x test point
     * @return probability density at x
     */
    static double density(const double (&mean)[2], const double (&covariance)[2][2], double heading, double width, double length, const double (&x)[2]) {
//        static const double inv2Pi = 0.1591549430918953; // square and square root cancel out
//        double det = covariance[0][0] * covariance[1][1] - covariance[0][1] * covariance[1][0];
//        assert(std::isfinite(det) && det > 0);
//        double inverseCovariance[2][2] = {
//                {covariance[1][1] / det, -covariance[0][1] / det},
//                {-covariance[1][0] / det, covariance[0][0] / det}
//        };
//        double xMinusMean[2] = {x[0] - mean[0], x[1] - mean[1]};
//        double intermediate[2] = { xMinusMean[0] * inverseCovariance[0][0] + xMinusMean[1] * inverseCovariance[0][1],
//                                   xMinusMean[1] * inverseCovariance[1][0] + xMinusMean[1] * inverseCovariance[1][1]};
//        double quadform = intermediate[0] * xMinusMean[0] + intermediate[1] * xMinusMean[1]; // (x - mu)T * Sigma^-1 * (x - mu)
//        assert(std::isfinite(quadform) && quadform >= 0);
//        double mahalanobisDistance = sqrt(quadform);
//        // Truncate distribution at two "standard deviations" (generalized to multivariate case by Mahalanobis distance)
//        if (mahalanobisDistance > 2) return 0;
//        double norm = inv2Pi / sqrt(det);
//        assert(std::isfinite(norm));
//        return norm * exp(-0.5 * quadform);
        auto theta = M_PI_2 - heading;
        width /= 2;
        length /= 2;
        auto translated_x = x[0] - mean[0];
        auto translated_y = x[1] - mean[1];
        auto rotated_x = translated_x * cos(theta) - translated_y * sin(theta);
        auto rotated_y = translated_x * sin(theta) + translated_y * cos(theta);
        if (fabs(rotated_x) < width && fabs(rotated_y) < length) return 1;
        else return 0;
    }
};


#endif //SRC_DISTRIBUTION_H
