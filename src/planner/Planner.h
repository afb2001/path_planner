#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H


#include <vector>
#include <path_planner/State.h>
#include "common/Map.h"
#include "search/Vertex.h"
#include "common/Path.h"
#include "common/Plan.h"

class Planner {
public:
    Planner(double maxSpeed, double maxTurningRadius, const Map& staticMap);

    void addToCover(const std::vector<std::pair<double, double>>& points);

    void clearToCover();

    virtual std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                                    DynamicObstacles dynamicObstacles, double timeRemaining);

    Plan tracePlan(const std::shared_ptr<Vertex>& v, bool smoothing, DynamicObstacles* obstacles);

protected:
    double m_MaxSpeed, m_MaxTurningRadius;
    Path m_PointsToCover;

    Map m_Map;

    std::ostream* m_Output = &std::cerr;

    double now() const;

};


#endif //SRC_PLANNER_H
