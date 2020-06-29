#ifndef SRC_DYNAMICOBSTACLESMANAGER_H
#define SRC_DYNAMICOBSTACLESMANAGER_H

#include <memory>
#include <path_planner_common/State.h>

/**
 * Interface for managing dynamic obstacles.
 */
class DynamicObstaclesManager {
public:
    typedef std::shared_ptr<DynamicObstaclesManager> SharedPtr;

    virtual ~DynamicObstaclesManager() = default;

    /**
     * Return a number weighted by increasing chance of collision. Not a probability, necessarily. Good luck tuning this.
     * @param x
     * @param y
     * @param time
     * @return not a probability
     */
    virtual double collisionExists(double x, double y, double time) const { return 0; };

    // convenience overload
    double collisionExists(const State& s) const {
        return collisionExists(s.x(), s.y(), s.time());
    };


};


#endif //SRC_DYNAMICOBSTACLESMANAGER_H
