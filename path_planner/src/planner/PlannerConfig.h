#ifndef SRC_PLANNERCONFIG_H
#define SRC_PLANNERCONFIG_H

#include <functional>
#include <assert.h>
#include "utilities/Visualizer.h"
#include "../common/map//Map.h"
#include "../common/dynamic_obstacles/DynamicObstaclesManager.h"

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

    double timeHorizon() const {
        return m_TimeHorizon;
    }

    void setTimeHorizon(double timeHorizon) {
        m_TimeHorizon = timeHorizon;
    }

    bool useBrownPaths() const {
        return m_UseBrownPaths;
    }

    void setUseBrownPaths(bool useBrownPaths) {
        m_UseBrownPaths = useBrownPaths;
    }

    int initialSamples() const {
        return m_InitialSamples;
    }

    void setInitialSamples(int initialSamples) {
        m_InitialSamples = initialSamples;
    }

    double collisionCheckingIncrement() const {
        return m_CollisionCheckingIncrement;
    }

    void setCollisionCheckingIncrement(double collisionCheckingIncrement) {
        m_CollisionCheckingIncrement = collisionCheckingIncrement;
    }

    double timeMinimum() const {
        return m_TimeMinimum;
    }

    void setTimeMinimum(double timeMinimum) {
        m_TimeMinimum = timeMinimum;
    }

private:
    // search branching factor
    int m_BranchingFactor = 9;
    // vehicle configuration (radii for Dubins model)
    double m_MaxSpeed = 2.5, m_TurningRadius = 8, m_CoverageTurningRadius = 16;
    // time horizon and minimum plan duration
    double m_TimeHorizon = 30, m_TimeMinimum = 5;
    // increment at which plans are collision checked (m)
    // TODO! -- really oughta be in seconds because when we have different speeds slower will accrue less collision
    //  penalty per second of being within an obstacle's probability distribution
    double m_CollisionCheckingIncrement = 1;
    // initial number of samples each planning iteration
    int m_InitialSamples = 100;
    // whether or not to be clever about getting onto the ribbon with some hand-picked curves
    bool m_UseBrownPaths = false;
    // whether to dump the motion tree to a file. tends to make search go a little slower, and files get big fast
    bool m_Visualizations = false;
    Visualizer::UniquePtr* m_Visualizer;
    // I don't remember why this was a good idea but it works well enough and I'm too scared to change it
    std::ostream** m_VisualizationStream = nullptr; // pointer to a pointer so we can change streams across copies
    // static map
    Map::SharedPtr m_Map;
    // dynamic obstacles
    DynamicObstaclesManager m_Obstacles;
    // Stream for output. Maybe this should go to its own ROS topic?
    std::ostream* m_Output;
    // function we pass in to let the planner check the time
    std::function<double()> m_NowFunction;
    // handy place to keep track of the starting time this iteration
    double m_StartStateTime;

};

#endif //SRC_PLANNERCONFIG_H
