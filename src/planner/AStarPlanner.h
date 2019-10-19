#ifndef SRC_ASTARPLANNER_H
#define SRC_ASTARPLANNER_H

#include "SamplingBasedPlanner.h"

class AStarPlanner : public SamplingBasedPlanner {
public:

    AStarPlanner(double maxSpeed, double maxTurningRadius, std::shared_ptr<Map> staticMap);
    AStarPlanner(double maxSpeed, double maxTurningRadius, double coverageSpeed, double coverageTurningRadius, std::shared_ptr<Map> staticMap);
    
    ~AStarPlanner() override = default;

    std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                            DynamicObstaclesManager dynamicObstacles, double timeRemaining) override;

    std::vector<State> plan(const RibbonManager& ribbonManager, const State& start,
                            DynamicObstaclesManager dynamicObstacles, double timeRemaining) override;

protected:

    std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator() override;

//    int k() const override { return 9; }

    std::shared_ptr<Vertex> aStar(DynamicObstaclesManager* obstacles, double endTime);

    void expandToCoverSpecificSamples(Vertex::SharedPtr root, const std::vector<State>& samples, DynamicObstaclesManager* obstacles);

    static constexpr double c_InitialSamples = 1024;

private:
    std::vector<State> plan(const State& start, DynamicObstaclesManager dynamicObstacles, double timeRemaining);
};


#endif //SRC_ASTARPLANNER_H
