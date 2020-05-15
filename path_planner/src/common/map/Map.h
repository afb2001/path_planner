#ifndef SRC_MAP_H
#define SRC_MAP_H


#include <memory>

/**
 * Abstract class to represent a map. The maps need a slight re-write because the planner doesn't actually need the
 * unblocked distance.
 */
class Map {
public:
    typedef std::shared_ptr<Map> SharedPtr;

    virtual ~Map() = default;

    /**
     * Get the safe distance from the grid cell containing x and y to the nearest static obstacle. Deprecated because
     * we really just want a bitmap.
     * @param x
     * @param y
     * @return
     */
    virtual double getUnblockedDistance(double x, double y) const;
};


#endif //SRC_MAP_H
