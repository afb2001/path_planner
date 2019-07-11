#ifndef SRC_SAMPLINGBASEDPLANNER_H
#define SRC_SAMPLINGBASEDPLANNER_H


#include "Planner.h"
#include "common/StateGenerator.h"
#include <functional>

class SamplingBasedPlanner : public Planner {
public:
    SamplingBasedPlanner(double maxSpeed, double maxTurningRadius, const Map& staticMap);

    std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
                            DynamicObstacles dynamicObstacles, double timeRemaining) override;

protected:
    double m_StartStateTime;
    std::vector<State> m_Samples;
    int m_ExpandedCount = 0;

    void pushVertexQueue(const std::shared_ptr<Vertex>& vertex);

    std::shared_ptr<Vertex> popVertexQueue();

    void clearVertexQueue();

    virtual std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator();

    bool goalCondition(const std::shared_ptr<Vertex>& vertex);

    virtual void expand(const std::shared_ptr<Vertex>& sourceVertex, DynamicObstacles* obstacles);

    virtual int k() const;

    void addSamples(StateGenerator& generator);
    void addSamples(StateGenerator& generator, int n);

private:
    std::vector<std::shared_ptr<Vertex>> m_VertexQueue;

    std::function<bool(const State& s1, const State& s2)> getStateComparator(const State& origin);
};


#endif //SRC_SAMPLINGBASEDPLANNER_H
