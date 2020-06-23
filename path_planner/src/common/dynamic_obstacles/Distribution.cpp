#include <stdexcept>
#include "Distribution.h"

Distribution::Distribution(const double (&mean)[2], const double (&covariance)[2][2], double width, double length,
        double heading, double time) : m_Time(time), m_Heading(heading), m_Width(width), m_Length(length) {
    for (int i = 0; i < 2; i++) {
        m_Mean[i] = mean[i];
        for (int j = 0; j < 2; j++) m_Covariance[i][j] = covariance[i][j];
    }
}

double Distribution::density(const Distribution& next, double desiredTime, const double (&x)[2]) const {
    auto interpolated = interpolate(next, desiredTime);
    return density(interpolated.m_Mean, interpolated.m_Covariance, m_Heading, m_Width, m_Length, x);
}

double Distribution::density(const Distribution& next, double desiredTime, double x, double y) const {
    double test[2] = {x, y};
    return density(next, desiredTime, test);
}

double Distribution::time() const {
    return m_Time;
}

Distribution Distribution::interpolate(const Distribution& next, double desiredTime) const {
    double mean[2];
    double covariance[2][2];
    double timeDiff = next.m_Time - m_Time;
    double desiredTimeDiff = desiredTime - m_Time;
    if (timeDiff == 0) {
        if (desiredTimeDiff == 0) {
            return {m_Mean, m_Covariance, m_Width, m_Length, m_Heading, desiredTime};
        } else {
            throw std::logic_error("Cannot interpolate between or extrapolate from two distributions with the same time stamp");
        }
    }
    for (int i = 0; i < 2; i++) {
        auto slope = (next.m_Mean[i] - m_Mean[i]) / timeDiff;
        mean[i] = m_Mean[i] + slope * desiredTimeDiff;
        for (int j = 0; j < 2; j++) {
            slope = (next.m_Covariance[i][j] - m_Covariance[i][j]) / timeDiff;
            covariance[i][j] = m_Covariance[i][j] + slope * desiredTimeDiff;
        }
    }
    double headingRate = (next.m_Heading - m_Heading) / timeDiff;
    auto heading = m_Heading + headingRate * desiredTimeDiff;
    return {mean, covariance, m_Width, m_Length, heading, desiredTime};
}

const double  (&Distribution::mean() const)[2] {
    return m_Mean;
}

double Distribution::density(const double (&x)[2]) const {
    return density(m_Mean, m_Covariance, m_Heading, m_Width, m_Length, x);
}

double Distribution::density(double x, double y) const {
    double test[2] = {x, y};
    return density(test);
}

double Distribution::heading() const {
    return m_Heading;
}