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

    // Ribbon width (on one side, so really half the full width)
    static constexpr double c_RibbonWidth = 2;
    // It might be a good idea for this to be the same as ribbon width. If it's smaller, you'll need to take another
    // look at the edge cost toCoverDistance calculation (should subtract ribbon width).
    // Intuitively, I think they should be the same, since you can cover a line going along it or across it.
    static constexpr double c_MinLength = c_RibbonWidth;
    static constexpr double c_MinSquaredLength = c_MinLength * c_MinLength;

    // use a tolerance to compensate for floating point errors that were happening
    static constexpr double c_Tolerance = 1e-5;



    double squaredLength() const {
        return (m_EndX - m_StartX) * (m_EndX - m_StartX) + (m_EndY - m_StartY) * (m_EndY - m_StartY);
    }
};


#endif //SRC_RIBBON_H
