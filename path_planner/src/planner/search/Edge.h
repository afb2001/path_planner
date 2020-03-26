#ifndef SRC_EDGE_H
#define SRC_EDGE_H

#include "Vertex.h"
#include "../../common/map/Map.h"
#include "../../common/dynamic_obstacles/DynamicObstaclesManager.h"
#include "../utilities/Path.h"
#include <path_planner_common/DubinsPlan.h>
#include "../PlannerConfig.h"
#include "../utilities/Ribbon.h"

extern "C" {
#include "dubins.h"
}
class Vertex;

class Edge {
public:
    typedef std::shared_ptr<Edge> SharedPtr;

    DubinsPath dubinsPath;
//    RobustDubins::Path dubinsPath;

    Edge(std::shared_ptr<Vertex> start, bool useRibbons);

    ~Edge();

    double approxCost() const;

    std::shared_ptr<Vertex> setEnd(const State& state);

    double computeTrueCost(const Map::SharedPtr& map, const DynamicObstaclesManager& obstacles, double maxSpeed, double maxTurningRadius);
    double computeTrueCost(const Map::SharedPtr& map, const DynamicObstaclesManager& obstacles);
    double computeTrueCost(const PlannerConfig& config);

    double trueCost() const;

    double computeApproxCost(double maxSpeed, double turningRadius);
    double computeApproxCost();

    void computeBrownPath(const PlannerConfig& config, const Ribbon& r);

    void smooth(Map::SharedPtr map, const DynamicObstaclesManager& obstacles, double maxSpeed, double maxTurningRadius);

    DubinsWrapper getPlan(const PlannerConfig& config);

    std::shared_ptr<Vertex> start() const;

    std::shared_ptr<Vertex> end() const;

    bool infeasible() const;

    double getSavedCollisionPenalty() const { return m_CollisionPenalty; }

    static double collisionPenalty() { return c_CollisionPenalty; }
    static double dubinsIncrement() { return c_DubinsIncrement; }
    static double timePenalty() { return c_TimePenalty; }

private:
    std::shared_ptr<Vertex> m_Start;
    std::weak_ptr<Vertex> m_End;

    DubinsWrapper m_DubinsWrapper;

    bool m_Infeasible = false;

    static constexpr double c_CollisionPenalty = 10; // no idea how to set this but this is probably too low (try 600)
    static constexpr double c_DubinsIncrement = 0.1;
    static constexpr double c_TimePenalty = 1;

    double m_ApproxCost = -1, m_TrueCost = -1;

    bool m_UseRibbons;

    double m_CollisionPenalty = 0;

    double netTime();
};


#endif //SRC_EDGE_H
