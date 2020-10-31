#ifndef SRC_MAP_H
#define SRC_MAP_H

#include <memory>
#include <cfloat>

/**
 * Base class to represent a map. This class has only the default implementation (nowhere is blocked).
 */
class Map {
public:
    typedef std::shared_ptr<Map> SharedPtr;

    virtual ~Map() = default;

    /**
     * Access the occupancy bitmap at the given location (map coordinates).
     * @param x
     * @param y
     * @return true iff the point is blocked
     */
    virtual bool isBlocked(double x, double y) const;

    /**
     * Get the bounding rectangle of the map (minX, maxX, minY, maxY). These are +/- double max by default.
     * @return array of length 4 containing extremes of the map
     */
    virtual const double* extremes() const;

    virtual double resolution() const;

private:
    double m_Extremes[4] = {-DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX};
};


#endif //SRC_MAP_H
