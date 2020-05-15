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

    /**
     * Launch a new thread for the plan loop.
     */
    void startPlanner();

    /**
     * Tell the planning thread to terminate.
     */
    void cancelPlanner();

    void refreshMap(std::string pathToMapFile, double latitude, double longitude);

    void pause();

    static double getCurrentTime();

    void setVehicleConfiguration(double turningRadius, double coverageTurningRadius, double maxSpeed, double lineWidth, int k, int heuristic);

    void setPlannerVisualization(bool visualize, const std::string& visualizationFilePath);



private:

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
    double m_LastUpdateTime = 1;
    double m_LastHeading = 0; // TODO! -- use moving average or something
    State m_LastState;

    // TODO! -- use ROS_INFO
    PlannerConfig m_PlannerConfig = PlannerConfig(&std::cerr);

    Visualizer::UniquePtr m_Visualizer;

    DynamicObstaclesManager m_DynamicObstaclesManager;

    std::mutex m_PauseMutex;
//    std::condition_variable m_PauseCv;

    std::shared_ptr<Map> m_NewMap = nullptr;
    std::string m_CurrentMapPath = "";
    std::mutex m_MapMutex;

    std::future<void> m_PlanningFuture;

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

    void planLoop();

    static std::vector<Distribution> inventDistributions(State obstacle);
};

#endif //SRC_EXECUTIVE_H
