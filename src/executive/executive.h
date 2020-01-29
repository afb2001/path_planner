#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include <condition_variable>
#include "../planner/utilities/RibbonManager.h"
#include "../trajectory_publisher.h"
#include "../planner/Planner.h"
#include <future>
#include <fstream>

class Executive
{
public:

    explicit Executive(TrajectoryPublisher *controlReceiver);

    ~Executive();

    void updateCovered(double x, double y, double speed, double heading, double t);

    void addRibbon(double x1, double y1, double x2, double y2);

    void clearRibbons();

    void updateDynamicObstacle(uint32_t mmsi, State obstacle);

    void startPlanner();

    void cancelPlanner();

    void refreshMap(std::string pathToMapFile, double latitude, double longitude);

    void pause();

    static double getCurrentTime();

    void setVehicleConfiguration(double maxSpeed, double turningRadius, double coverageMaxSpeed, double coverageTurningRadius, int k);

    void setPlannerVisualization(bool visualize, const std::string& visualizationFilePath);



private:

    bool m_PlannerCancelled = false;
    std::mutex m_CancelLock;
    std::condition_variable m_CancelCV;

    RibbonManager m_RibbonManager;
    double m_LastUpdateTime = 1;
    double m_LastHeading = 0; // TODO! -- use moving average or something
    State m_LastState;

    // TODO! -- use ROS_INFO
    PlannerConfig m_PlannerConfig = PlannerConfig(&std::cerr);

    Visualizer::UniquePtr m_Visualizer;

    DynamicObstaclesManager m_DynamicObstaclesManager;

    bool m_Pause = true;

    std::mutex m_PauseMutex;
    std::condition_variable m_PauseCv;

    std::shared_ptr<Map> m_NewMap = nullptr;
    std::string m_CurrentMapPath = "";
    std::mutex m_MapMutex;

    std::future<void> m_PlanningFuture;

    TrajectoryPublisher* m_TrajectoryPublisher;

    static constexpr double c_CoverageHeadingRateMax = 0.1; // (in radians/sec)
    static constexpr double c_PlanningTimeSeconds = 1;

    /**
     * Clear m_PauseAll and notify threads blocked on it.
     */
    void unPause();

    /**
     * Make sure the threads can exit and kill the planner (if it's running).
     */
    void terminate();

    void planLoop();

    static std::vector<Distribution> inventDistributions(State obstacle);
};

#endif //SRC_EXECUTIVE_H
