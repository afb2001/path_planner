#ifndef SRC_MAP_H
#define SRC_MAP_H


#include <memory>

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
};


#endif //SRC_MAP_H
