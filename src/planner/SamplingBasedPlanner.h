#ifndef SRC_SAMPLINGBASEDPLANNER_H
#define SRC_SAMPLINGBASEDPLANNER_H


#include "Planner.h"
#include "common/StateGenerator.h"

class SamplingBasedPlanner : public Planner {
public:
    SamplingBasedPlanner(double maxSpeed, double maxTurningRadius, const Map& staticMap);

    std::vector<State> plan(const std::vector<std::pair<double, double>> &newlyCovered,
                                    const State &start, DynamicObstacles dynamicObstacles) override;

protected:
    double m_StartStateTime;
    std::vector<State> m_Samples;

    void pushVertexQueue(const std::shared_ptr<Vertex>& vertex);

    std::shared_ptr<Vertex> popVertexQueue();

    virtual std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator();

    bool goalCondition(const std::shared_ptr<Vertex>& vertex);

    void expand(const std::shared_ptr<Vertex>& sourceVertex, DynamicObstacles* obstacles);

    virtual int k() const;

    void addSamples(StateGenerator& generator);
    void addSamples(StateGenerator& generator, int n);

private:
    std::vector<std::shared_ptr<Vertex>> m_VertexQueue;

    static std::function<bool(const State& s1, const State& s2)> getStateComparator(const State& origin);
};


#endif //SRC_SAMPLINGBASEDPLANNER_H
