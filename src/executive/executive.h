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

    static constexpr double DefaultMaxSpeed = 2.3;
    static constexpr double DefaultTurningRadius = 8;

private:

    bool m_Running = false;
    ExecutiveInternalsManager m_InternalsManager;
    RibbonManager m_RibbonManager;
    double m_LastUpdateTime = 1;
    double m_LastHeading = 0; // TODO! -- use moving average or something

    DynamicObstaclesManager m_DynamicObstaclesManager;

    bool m_Pause = true;

    mutex m_PauseMutex;
    condition_variable m_PauseCv;

    std::unique_ptr<Planner> m_Planner;

    std::shared_ptr<Map> m_NewMap = nullptr;
    mutex m_MapMutex;

    std::future<void> m_TrajectoryPublishingFuture, m_PlanningFuture;

    TrajectoryPublisher* m_TrajectoryPublisher;

    static constexpr double c_CoverageHeadingRateMax = 0.1; // (in radians/sec)

    void requestPath();

    void sendAction();

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
