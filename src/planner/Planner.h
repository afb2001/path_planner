#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H


#include <vector>
#include <path_planner/State.h>
#include "../common/map/Map.h"
#include "search/Vertex.h"
#include "utilities/Path.h"
#include "utilities/Plan.h"

class Planner {
public:
    Planner(double maxSpeed, double maxTurningRadius, Map::SharedPtr staticMap);

    virtual ~Planner() = default;

    void addToCover(const std::vector<std::pair<double, double>>& points);

    void clearToCover();

    virtual std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                                    DynamicObstaclesManager dynamicObstacles, double timeRemaining);

    virtual std::vector<State> plan(const RibbonManager& ribbonManager, const State& start,
            DynamicObstaclesManager dynamicObstacles, double timeRemaining);

    std::vector<State> plan(const RibbonManager& ribbonManager, const State& start,
            DynamicObstaclesManager dynamicObstacles, double timeRemaining, double maxSpeed, double turningRadius);

    Plan tracePlan(const std::shared_ptr<Vertex>& v, bool smoothing, DynamicObstaclesManager* obstacles);

    void updateMap(Map::SharedPtr map);

protected:
    double m_MaxSpeed, m_TurningRadius;
    Path m_PointsToCover;

    Map::SharedPtr m_Map;

    std::ostream* m_Output = &std::cerr;

    double now() const;

};


#endif //SRC_PLANNER_H
