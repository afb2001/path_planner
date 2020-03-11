#ifndef SRC_SAMPLINGBASEDPLANNER_H
#define SRC_SAMPLINGBASEDPLANNER_H

#include "Planner.h"
#include "utilities/StateGenerator.h"
#include <functional>

class SamplingBasedPlanner : public Planner {
public:
    SamplingBasedPlanner();

    ~SamplingBasedPlanner() override = default;

//    std::vector<State> plan(const std::vector<std::pair<double, double>>& newlyCovered, const State& start,
//                            DynamicObstaclesManager dynamicObstacles, double timeRemaining) override;
//
//    std::vector<State> plan(const RibbonManager& ribbonManager, const State& start,
//                            DynamicObstaclesManager dynamicObstacles, double timeRemaining) override;

    Plan plan(const RibbonManager&, const State& start, PlannerConfig config,
            double timeRemaining) override;

    void pushVertexQueue(Vertex::SharedPtr vertex);

    std::shared_ptr<Vertex> popVertexQueue();

    void clearVertexQueue();

    virtual void expand(const std::shared_ptr<Vertex>& sourceVertex, const DynamicObstaclesManager& obstacles);

    void addSamples(StateGenerator& generator);
    void addSamples(StateGenerator& generator, int n);

//    void setRibbonManager(const RibbonManager& ribbonManager);

//    void setK(int k) override;

protected:
    double m_StartStateTime;
    std::vector<State> m_Samples;
    int m_ExpandedCount = 0;

    Vertex::SharedPtr m_BestVertex;

//    int m_K;

//    bool m_UseRibbons = false;
    RibbonManager m_RibbonManager;

    virtual std::function<bool(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)> getVertexComparator();

    bool goalCondition(const std::shared_ptr<Vertex>& vertex);

    virtual int k() const;

    void visualizeVertex(Vertex::SharedPtr v, const std::string& tag);

    void visualizeRibbons(const RibbonManager& ribbonManager);

    bool vertexQueueEmpty() const;

private:
    std::vector<std::shared_ptr<Vertex>> m_VertexQueue;

    std::function<bool(const State& s1, const State& s2)> getStateComparator(const State& origin);

    std::function<bool(const std::shared_ptr<Vertex>&, const std::shared_ptr<Vertex>&)> getDubinsComparator(
            const State& origin);

//    std::vector<State> plan(const State& start, DynamicObstaclesManager dynamicObstacles, double timeRemaining);
};


#endif //SRC_SAMPLINGBASEDPLANNER_H
