#ifndef SRC_ASTARPLANNER_H
#define SRC_ASTARPLANNER_H

#include "SamplingBasedPlanner.h"

class AStarPlanner : public SamplingBasedPlanner {
public:

    AStarPlanner(double maxSpeed, double maxTurningRadius, std::shared_ptr<Map> staticMap);

    std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                            DynamicObstaclesManager dynamicObstacles, double timeRemaining) override;

protected:

    std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator() override;

    int k() const override { return 17; }

    std::shared_ptr<Vertex> aStar(DynamicObstaclesManager* obstacles, double endTime);

    static constexpr double c_InitialSamples = 32;
};


#endif //SRC_ASTARPLANNER_H
