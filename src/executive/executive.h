#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include <string.h>
#include <condition_variable>
#include "communication.h"
#include "path.h"
#include "../trajectory_publisher.h"


/**
 * This class manages the planner and exposes member functions to update its information.
 */
class Executive
{
public:

    /**
     * Construct an Executive instance.
     * @param trajectoryPublisher an object that can publish trajectories.
     */
    explicit Executive(TrajectoryPublisher *trajectoryPublisher);

    ~Executive();

    /**
     * Update the state of the vehicle and check whether we've covered any new points.
     * @param x
     * @param y
     * @param speed
     * @param heading
     * @param t
     */
    void updateCovered(double x, double y, double speed, double heading, double t);

    /**
     * Add a point to cover at <x, y>
     * @param x
     * @param y
     */
    void addToCover(int x, int y);

    /**
     * Update the dynamic obstacle mmsi with a new observation.
     * @param mmsi the dynamic obstacle's mmsi
     * @param obstacle the new observation of the obstacle
     */
    void updateDyamicObstacle(uint32_t mmsi, State obstacle);

    /**
     * Start the planner with the given map file. For an empty map use mapFile="NOFILE".
     * @param mapFile the path to a grid-world-style map file
     */
    void startPlanner(std::string mapFile);

    /**
     * @return whether the planner is still running
     */
    bool plannerIsRunning();

    /**
     * Stop the planner and pause everything else.
     */
    void pause();

private:

    bool m_Running = false;
    bool m_Pause = true;
    bool m_PlannerPipeStale = true;

    Path m_Path;

    std::mutex m_PauseMutex;
    std::condition_variable m_PauseCV;

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
