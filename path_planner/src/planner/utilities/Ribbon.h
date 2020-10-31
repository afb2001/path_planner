#ifndef SRC_RIBBON_H
#define SRC_RIBBON_H

#include <path_planner_common/State.h>

#include <cmath>
#include <utility>
#include <string>

/**
 * Represent a survey line as a line segment with a width (ribbon).
 */
class Ribbon {
public:
    // Ribbon width (on one side, so really half the full width).
    // I tried to stay away from public static members but it'd be much more work to avoid it here.
    static double RibbonWidth;

    /**
     * Construct a ribbon from (startX, startY) to (endX, endY).
     * @param startX
     * @param startY
     * @param endX
     * @param endY
     */
    Ribbon(double startX, double startY, double endX, double endY);

    /**
     * Split a ribbon at (x, y).
     * @param x
     * @param y
     * @return
     */
    Ribbon split(double x, double y, bool strict);

    /**
     * @return true iff the ribbon is below the minimum length (all covered).
     */
    bool covered(bool strict) const;

    /**
     * Construct an empty ribbon.
     * @return
     */
    static Ribbon empty();

    /**
     * @return the length of the ribbon.
     */
    double length() const;

    /**
     * @return the start as a pair
     */
    std::pair<double, double> start() const;

    /**
     * @return the end as a pair
     */
    std::pair<double, double> end() const;

    /**
     * @return the start as a state
     */
    State startAsState() const;

    /**
     * @return the end as a state
     */
    State endAsState() const;

    /**
     * Determine whether (x, y) is contained within the ribbon, using a pre-computed projection.
     *
     * This is split up because the projection is useful elsewhere.
     *
     * @param x
     * @param y
     * @param projected
     * @return
     */
    bool contains(double x, double y, const std::pair<double, double>& projected, bool strict) const;

    /**
     * Determine whether a point already projected onto the ribbon line is contained within the ribbon.
     * @param projected
     * @return
     */
    bool containsProjection(const std::pair<double, double>& projected) const;

    /**
     * @return a string representation of the ribbon
     */
    std::string toString() const;

    /**
     * The minimum ribbon length, based on the width
     * @return
     */
    static double minLength();

    /**
     * Project a point onto the ribbon.
     * @param x
     * @param y
     * @return
     */
    std::pair<double, double> getProjection(double x, double y) const;

    /**
     * Get a projection of a point onto the ribbon as a state.
     * @param x
     * @param y
     * @return
     */
    State getProjectionAsState(double x, double y) const;

    // perpendicular distance to line through (startX, startY), (endX, endY)
    double distance(double x, double y) const {
        return (std::fabs((m_EndY - m_StartY) * x - (m_EndX - m_StartX) * y + m_EndX*m_StartY - m_EndY*m_StartX)) /
                std::sqrt(squaredLength());
    }

    static constexpr double strictModifier() { return c_StrictModifier; }

private:
    double m_StartX, m_StartY, m_EndX, m_EndY;

    // use a tolerance to compensate for floating point errors that were happening
    static constexpr double c_Tolerance = 1e-5;

    static constexpr double c_StrictModifier = 2;

    double squaredLength() const {
        return (m_EndX - m_StartX) * (m_EndX - m_StartX) + (m_EndY - m_StartY) * (m_EndY - m_StartY);
    }
};


#endif //SRC_RIBBON_H
