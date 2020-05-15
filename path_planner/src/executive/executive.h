#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include <condition_variable>
#include "../planner/utilities/RibbonManager.h"
#include "../trajectory_publisher.h"
#include "../planner/Planner.h"
#include <future>
#include <fstream>

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
    void updateDynamicObstacle(uint32_t mmsi, State obstacle);
    void updateDynamicObstacle(uint32_t mmsi, const std::vector<Distribution>& obstacle);

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
    void setConfiguration(double turningRadius, double coverageTurningRadius, double maxSpeed, double lineWidth, int k, int heuristic);

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

    PlannerState m_PlannerState = PlannerState::Inactive;
    std::mutex m_PlannerStateMutex;
    std::condition_variable m_CancelCV;

    std::mutex m_RibbonManagerMutex;
    RibbonManager m_RibbonManager;
    double m_LastUpdateTime = 1; // could use the time in m_LastState I think but this is cleaner
    double m_LastHeading = 0; // TODO! -- use moving average or something
    State m_LastState;

    // TODO! -- use ROS_INFO
    PlannerConfig m_PlannerConfig = PlannerConfig(&std::cerr);

    Visualizer::UniquePtr m_Visualizer;

    DynamicObstaclesManager m_DynamicObstaclesManager;

    // start with no new map
    std::shared_ptr<Map> m_NewMap = nullptr;
    std::string m_CurrentMapPath = "";
    std::mutex m_MapMutex;

    // hold onto the thread doing planning, for elegant error handling and shutdown I guess
    std::future<void> m_PlanningFuture;

    // pointer to the ROS node. Should this be a reference? I didn't know how to use references correctly when I wrote this
    TrajectoryPublisher* m_TrajectoryPublisher;

    double m_RadiusShrink = 0;

    static constexpr bool c_RadiusShrinkEnabled = false;
    static constexpr double c_RadiusShrinkAmount = 1e-6;

    static constexpr bool c_ReusePlanEnabled = true;
    static constexpr double c_CoverageHeadingRateMax = 0.1; // (in radians/sec)
    static constexpr double c_PlanningTimeSeconds = 1;

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
    static std::vector<Distribution> inventDistributions(State obstacle);
};

#endif //SRC_EXECUTIVE_H
