#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include "../planner/utilities/RibbonManager.h"
#include "../trajectory_publisher.h"
#include "../planner/Planner.h"
#include "../common/dynamic_obstacles/BinaryDynamicObstaclesManager.h"
#include "../common/dynamic_obstacles/GaussianDynamicObstaclesManager.h"
#include <path_planner_common/State.h>

#include <condition_variable>
#include <cstdio> // stderr
#include <fstream>
#include <future>
#include <memory>
#include <string>

/**
 * Class calls the planner and manages associated configurations and other data.
 *
 * The planner runs in its own thread. Its information gets updated asynchronously through this interface by the ROS node.
 * The trajectory returned by the planner gets sent to the controller via a service call, the response of which contains
 * the start state for the next planning iteration.
 */
class Executive
{
public:

    /**
     * Construct an executive object. This probably only needs to happen once per ROS node.
     * @param trajectoryPublisher - Intended to be a ROS node
     */
    explicit Executive(TrajectoryPublisher *trajectoryPublisher);

    /**
     * Attempts to stop the planning thread. Waits up to two seconds.
     */
    ~Executive();

    /**
     * Update the coverage with a new vehicle pose.
     * @param x
     * @param y
     * @param speed
     * @param heading
     * @param t
     */
    void updateCovered(double x, double y, double speed, double heading, double t);

    /**
     * Add a new survey line.
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     */
    void addRibbon(double x1, double y1, double x2, double y2);

    /**
     * Clear the survey lines.
     */
    void clearRibbons();

    /**
     * Update information about a dynamic obstacle.
     * @param mmsi
     * @param obstacle
     */
    void updateDynamicObstacle(uint32_t mmsi, State obstacle, double width, double length);
//    void updateDynamicObstacle(uint32_t mmsi, const std::vector<Distribution>& obstacle);

    /**
     * Launch a new thread for the plan loop.
     */
    void startPlanner();

    /**
     * Tell the planning thread to terminate.
     */
    void cancelPlanner();

    /**
     * Re-load the map from the given path.
     * @param pathToMapFile
     * @param latitude origin latitude
     * @param longitude origin longitude
     */
    void refreshMap(const std::string& pathToMapFile, double latitude, double longitude);

    /**
     * Utility to get the current time. Public for testing, and only used when disconnected from ROS.
     * @return
     */
    static double getCurrentTime();

    /**
     * Update the configuration of the planner.
     * @param turningRadius
     * @param coverageTurningRadius
     * @param maxSpeed
     * @param lineWidth
     * @param k
     * @param heuristic
     */
    void setConfiguration(double turningRadius, double coverageTurningRadius, double maxSpeed, double slowSpeed,
                          double lineWidth, int k, int heuristic, double timeHorizon, double timeMinimum,
                          double collisionCheckingIncrement, int initialSamples, bool useBrownPaths,
                          bool useGaussianDynamicObstacles, bool ignoreDynamicObstacles, bool usePotentialField);

    /**
     * Update the planner visualization status with a new visualization file. If visualize is false the path is ignored.
     * @param visualize
     * @param visualizationFilePath
     */
    void setPlannerVisualization(bool visualize, const std::string& visualizationFilePath);

private:

    /**
     * Enum to represent the planner's state. This is part of my solution to some tricky concurrency problems arising
     * from the asynchronicity of planning.
     */
    enum PlannerState {
        Inactive,
        Running,
        Cancelled,
    };

    // handling of planner state. This is mostly for making sure threads exit properly when canceled and start again correctly
    PlannerState m_PlannerState = PlannerState::Inactive;
    std::mutex m_PlannerStateMutex;
    std::condition_variable m_CancelCV;

    // info for coverage checking
    std::mutex m_RibbonManagerMutex;
    RibbonManager m_RibbonManager;
    double m_LastUpdateTime = 1; // could use the time in m_LastState I think but this is cleaner
    double m_LastHeading = 0; // TODO! -- use moving average or something
    State m_LastState;

    // TODO! -- use ROS_INFO
    PlannerConfig m_PlannerConfig = PlannerConfig(&std::cerr);

    // only two obstacle managers for now so it can be a bool
    bool m_UseGaussianDynamicObstacles = false;

    // whether or not to ignore dynamic obstacles
    bool m_IgnoreDynamicObstacles = false;

    // flag for the potential field planner (ignores most other planner config options)
    bool m_UsePotentialField = false;

    // handle on a visualizer. This could probably be handled in a cleaner way
    Visualizer::UniquePtr m_Visualizer;

//    DynamicObstaclesManager1 m_DynamicObstaclesManager;

    // Just keep both dynamic obstacle managers up to date all the time, regardless of which we're using
    BinaryDynamicObstaclesManager::SharedPtr m_BinaryDynamicObstaclesManager = std::make_shared<BinaryDynamicObstaclesManager>();
    GaussianDynamicObstaclesManager::SharedPtr m_GaussianDynamicObstaclesManager = std::make_shared<GaussianDynamicObstaclesManager>();

    // map info (start with no new map)
    std::shared_ptr<Map> m_NewMap = nullptr;
    std::string m_CurrentMapPath = "";
    std::mutex m_MapMutex;

    // hold onto the thread doing planning, for elegant error handling and shutdown I guess
    std::future<void> m_PlanningFuture;

    // pointer to the ROS node. Should this be a reference? I didn't know how to use references correctly when I wrote this
    TrajectoryPublisher* m_TrajectoryPublisher;

    // radius shrink total
    double m_RadiusShrink = 0;
    // other radius shrink info
    static constexpr bool c_RadiusShrinkEnabled = false;
    static constexpr double c_RadiusShrinkAmount = 1e-6;

    // whether or not to re-use plans (I don't see why you would ever not want this except debugging and testing)
    static constexpr bool c_ReusePlanEnabled = true;

    // heading rate constraint on coverage. This should really be looked at by someone who knows more about surveying
    static constexpr double c_CoverageHeadingRateMax = 0.1; // (in radians/sec)

    // Planning time constant. Would be nice to make this a parameter but the controller depends on it being 1s.
    // You might notice that this isn't actually 1s; I played around with it and this setting got closest to actually
    // giving us 1s times.
    static constexpr double c_PlanningTimeSeconds = 0.85;

    /**
     * Make sure the threads can exit and kill the planner (if it's running).
     */
    void terminate();

    /**
     * Run the planner in a loop.
     */
    void planLoop();

    /**
     * Nothing provides the distributions for dynamic obstacles yet so the executive invents them.
     * @param obstacle
     * @return
     */
//    static std::vector<Distribution> inventDistributions(State obstacle);
};

#endif //SRC_EXECUTIVE_H
