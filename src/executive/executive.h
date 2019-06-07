#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include <string.h>
#include <condition_variable>
#include "communication.h"
#include "path.h"
#include "../trajectory_publisher.h"


class Executive
{
public:

    explicit Executive(TrajectoryPublisher *controlReceiver);

    ~Executive();

    void updateCovered(double x, double y, double speed, double heading, double t);

    void addToCover(int x, int y);

    void updateDyamicObstacle(uint32_t mmsi, State obstacle);

    void startPlanner(std::string mapFile);

    bool plannerIsRunning();

    void pause();

private:

    bool m_Running = false;
    bool m_Pause = true;
    bool m_PlannerPipeStale = true;

    Path m_Path;

    mutex m_PauseMutex;
    condition_variable m_PauseCV;

    Communication m_PipeToPlanner;

    TrajectoryPublisher* m_TrajectoryPublisher;

    bool plannerIsDead();

    static double getCurrentTime();

    void requestPath();

// TODO! -- Add a way to update obstacles

    void sendAction();

    void print_map(std::string file);

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
};

#endif //SRC_EXECUTIVE_H
