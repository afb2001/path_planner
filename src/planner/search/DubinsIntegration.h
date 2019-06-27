#ifndef SRC_DUBINSINTEGRATION_H
#define SRC_DUBINSINTEGRATION_H

#include "../common/DynamicObstacles.h"
#include "../common/Map.h"
#include "../common/Path.h"
#include "Edge.h"
#include "../common/Plan.h"

extern "C" {
    #include <dubins.h>
}

#define DUBINS_INCREMENT 0.1
#define COLLISION_PENALTY 600
#define COVERAGE_THRESHOLD 3

//int dubins_path_sample_many(DubinsPath*, double, int (*)(double*, double, void*), void*);

/**
 * This class provides some functionality on top of the dubins library.
 * I wanted to get rid of this class in the port but I would have needed to rely on closures to modify local variables
 * in the callbacks and I couldn't get the types to line up (lambdas can't be DubinsSamplingCallbacks).
 */
class DubinsIntegration {
public:
    DubinsIntegration();

    std::vector<std::pair<double, double>>
    computeEdgeCollisionPenaltyAndNewlyCovered(DubinsPath path, Map *map, DynamicObstacles *obstacles, Path toCover,
                                               double &penalty);

    Plan getPlan(const State& start, DubinsPath dubinsPath, double maxSpeed);
private:
    Path m_ToCover;
    std::vector<std::pair<double, double>> m_NewlyCovered;
    Map* m_Map = nullptr;
    DynamicObstacles* m_DynamicObstacles = nullptr;
    double m_Penalty = 0, m_ObstacleDistance = 0, m_MaxSpeed = 0;
    Plan m_Plan;

    static int edgeCostCallback(double q[3], double accumulatedDistance, void *userData);

    static void addNewlyCovered(const Path& toCover, double x, double y,
                                std::vector<std::pair<double, double>> &covered);

    static int getPlanCallback(double q[3], double accumulatedDistance, void *userData);
};


#endif //SRC_DUBINSINTEGRATION_H
