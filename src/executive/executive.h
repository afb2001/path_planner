#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include <condition_variable>
#include "ExecutiveInternalsManager.h"
#include "../planner/utilities/RibbonManager.h"
#include "../trajectory_publisher.h"
#include "../planner/Planner.h"
#include <future>

class Executive
{
public:

    explicit Executive(TrajectoryPublisher *controlReceiver);

    ~Executive();

    void updateCovered(double x, double y, double speed, double heading, double t);

    void addToCover(int x, int y);
    void addRibbon(double x1, double y1, double x2, double y2);
    void clearRibbons();

    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    void startPlanner(const string& mapFile, double latitude, double longitude);

    void refreshMap(std::string pathToMapFile, double latitude, double longitude);

    bool plannerIsRunning();

    void pause();

    static double getCurrentTime();

//    double MaxSpeed = 2.3;
//    double TurningRadius = 8;
//    double CoverageMaxSpeed = 2.3;
//    double CoverageTurningRadius = 16;
//    int K = 9;

    void setVehicleConfiguration(double maxSpeed, double turningRadius, double coverageMaxSpeed, double coverageTurningRadius, int k);



private:

    bool m_Running = false;
    ExecutiveInternalsManager m_InternalsManager;
    RibbonManager m_RibbonManager;
    double m_LastUpdateTime = 1;
    double m_LastHeading = 0; // TODO! -- use moving average or something
    State m_LastState;

    // TODO! -- use ROS_INFO
    PlannerConfig m_PlannerConfig = PlannerConfig(&std::cerr);

    DynamicObstaclesManager m_DynamicObstaclesManager;

    bool m_Pause = true;

    mutex m_PauseMutex;
    condition_variable m_PauseCv;

    std::unique_ptr<Planner> m_Planner;

    std::shared_ptr<Map> m_NewMap = nullptr;
    std::string m_CurrentMapPath = "";
    mutex m_MapMutex;

    std::future<void> m_TrajectoryPublishingFuture, m_PlanningFuture;

    TrajectoryPublisher* m_TrajectoryPublisher;

    static constexpr double c_CoverageHeadingRateMax = 0.1; // (in radians/sec)
    static constexpr double c_PlanningTimeSeconds = 1;

    void requestPath();

    /**
     * Clear m_PauseAll and notify threads blocked on it.
     */
    void unPause();

    /**
     * Start threads for listening to the planner and updating the controller.
     */
    void startThreads();

    /**
     * Make sure the threads can exit and kill the planner (if it's running).
     */
    void terminate();

    static std::vector<Distribution> inventDistributions(State obstacle);
};

#endif //SRC_EXECUTIVE_H
