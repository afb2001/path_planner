#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H


#include <vector>
#include <path_planner/State.h>
#include "../common/map/Map.h"
#include "search/Vertex.h"
#include "utilities/Path.h"
#include "utilities/Plan.h"
#include "PlannerConfig.h"

class Planner {
public:
    Planner();

    virtual ~Planner() = default;

//    void addToCover(const std::vector<std::pair<double, double>>& points);

//    void clearToCover();

//    virtual std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
//                                    DynamicObstaclesManager dynamicObstacles, double timeRemaining);

//    virtual std::vector<State> plan(const RibbonManager& ribbonManager, const State& start,
//            DynamicObstaclesManager dynamicObstacles, double timeRemaining);

    virtual std::vector<State> plan(const RibbonManager& ribbonManager, const State& start,
                            PlannerConfig config, double timeRemaining);

    Plan tracePlan(const std::shared_ptr<Vertex>& v, bool smoothing, const DynamicObstaclesManager& obstacles);

    /**
     * Manually set the planner config. Meant for testing.
     * @param config
     */
    void setConfig(PlannerConfig config);

//    void updateMap(Map::SharedPtr map);

//    virtual void setK(int k);

protected:
//    double m_MaxSpeed, m_TurningRadius, m_CoverageMaxSpeed, m_CoverageTurningRadius;
//    Path m_PointsToCover;

//    Map::SharedPtr m_Map;

//    std::ostream* m_Output = &std::cerr;

    double now() const;

    PlannerConfig m_Config;

};


#endif //SRC_PLANNER_H
