#ifndef SRC_PLANNERCONFIG_H
#define SRC_PLANNERCONFIG_H

#include <functional>
#include <assert.h>
#include "utilities/Visualizer.h"

/**
 * Class that holds all the configurations for the planner. These need to get passed around periodically so it was
 * easiest to make a single object that holds them all. It just has getters and setters pretty much.
 */
class PlannerConfig {
public:

    explicit PlannerConfig(std::ostream* output) : m_Output(output) {}

    int branchingFactor() const {
        return m_BranchingFactor;
    }

    void setBranchingFactor(int branchingFactor) {
        m_BranchingFactor = branchingFactor;
    }

    double maxSpeed() const {
        return m_MaxSpeed;
    }

    void setMaxSpeed(double maxSpeed) {
        m_MaxSpeed = maxSpeed;
    }

    double turningRadius() const {
        return m_TurningRadius;
    }

    void setTurningRadius(double turningRadius) {
        m_TurningRadius = turningRadius;
    }

//    double coverageMaxSpeed() const {
//        return m_CoverageMaxSpeed;
//    }
//
//    void setCoverageMaxSpeed(double coverageMaxSpeed) {
//        m_CoverageMaxSpeed = coverageMaxSpeed;
//    }

    double coverageTurningRadius() const {
        return m_CoverageTurningRadius;
    }

    void setCoverageTurningRadius(double coverageTurningRadius) {
        m_CoverageTurningRadius = coverageTurningRadius;
    }

    bool visualizations() const {
        return m_Visualizations;
    }

    void setVisualizations(bool visualizations) {
        m_Visualizations = visualizations;
    }

    std::ostream& visualizationStream() const {
        assert(m_Visualizations && "Visualization stream accessed when visualizations are disabled");
//        assert(m_VisualizationStream && *m_VisualizationStream && "Visualization stream accessed but does not exist");
        return (*m_Visualizer)->stream();
    }

    void setVisualizationStream(std::ostream** visualizationStream) {
        if (m_VisualizationStream) {
            *m_VisualizationStream = *visualizationStream;
        } else {
            m_VisualizationStream = visualizationStream;
        }
    }

    const Map::SharedPtr& map() const {
        return m_Map;
    }

    void setMap(const Map::SharedPtr& map) {
        m_Map = map;
    }

    const DynamicObstaclesManager& obstacles() const {
        return m_Obstacles;
    }

    void setObstacles(const DynamicObstaclesManager& obstacles) {
        m_Obstacles = obstacles;
    }

    std::ostream* output() const {
        return m_Output;
    }

    void setNowFunction(const std::function<double()>& nowFunction) {
        m_NowFunction = nowFunction;
    }

    double now() const { return m_NowFunction(); }

    void setVisualizer(Visualizer::UniquePtr* visualizer) {
        m_Visualizer = visualizer;
    }

    double startStateTime() const {
        return m_StartStateTime;
    }

    void setStartStateTime(double startStateTime) {
        m_StartStateTime = startStateTime;
    }

private:
    int m_BranchingFactor = 9;
    double m_MaxSpeed = 2.5, m_TurningRadius = 8, m_CoverageTurningRadius = 16;
    bool m_Visualizations = false;
    Visualizer::UniquePtr* m_Visualizer;
    std::ostream** m_VisualizationStream = nullptr; // pointer to a pointer so we can change streams across copies
    Map::SharedPtr m_Map;
    DynamicObstaclesManager m_Obstacles;
    std::ostream* m_Output;
    std::function<double()> m_NowFunction;
    double m_StartStateTime;

};

#endif //SRC_PLANNERCONFIG_H
