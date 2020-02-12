#ifndef SRC_RIBBON_H
#define SRC_RIBBON_H

#include <cmath>
#include <utility>
#include <string>
#include <path_planner/State.h>

/**
 * Represent a survey line as a line segment with a width (ribbon).
 */
class Ribbon {
public:
    // Ribbon width (on one side, so really half the full width).
    // I tried to stay away from public static members but it'd be much more work to avoid it here.
    static double RibbonWidth;

    Ribbon(double startX, double startY, double endX, double endY);

    Ribbon split(double x, double y);

    bool covered() const;

    static Ribbon empty();

    double length() const;

    std::pair<double, double> start() const;

    std::pair<double, double> end() const;

    State startAsState() const;

    State endAsState() const;

    bool contains(double x, double y, const std::pair<double, double>& projected) const;

    bool containsProjection(const std::pair<double, double>& projected) const;

    std::string toString() const;

    static double minLength();

    std::pair<double, double> getProjection(double x, double y) const;

    State getProjectionAsState(double x, double y) const;

    // perpendicular distance to line through (startX, startY), (endX, endY)
    double distance(double x, double y) const {
        return (fabs((m_EndY - m_StartY) * x - (m_EndX - m_StartX) * y + m_EndX*m_StartY - m_EndY*m_StartX)) /
               sqrt(squaredLength());
    }

private:
    double m_StartX, m_StartY, m_EndX, m_EndY;

    // use a tolerance to compensate for floating point errors that were happening
    static constexpr double c_Tolerance = 1e-5;



    double squaredLength() const {
        return (m_EndX - m_StartX) * (m_EndX - m_StartX) + (m_EndY - m_StartY) * (m_EndY - m_StartY);
    }
};


#endif //SRC_RIBBON_H
