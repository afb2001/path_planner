#ifndef SRC_TRAJECTORYDISPLAYER_H
#define SRC_TRAJECTORYDISPLAYER_H

#include <path_planner_common/State.h>
#include <vector>

/**
 * Interface for displaying trajectories
 */
class TrajectoryDisplayer {
public:
    /**
     * Display a trajectory to /project11/display. Map coordinates are converted into lat/long with the map_to_wgs84
     * service.
     * @param trajectory the trajectory to display
     * @param plannerTrajectory flag determining color and id
     */
    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) {
        displayTrajectory(trajectory, plannerTrajectory, true);
//        for (const auto& s : trajectory) std::cerr << s.toString() << std::endl; // print trajectory
    }

    /**
     * Display a trajectory to /project11/display. Map coordinates are converted into lat/long with the map_to_wgs84
     * service.
     * @param trajectory the trajectory to display
     * @param plannerTrajectory flag determining color and id
     * @param achievable whether this trajectory is achievable
     */
    virtual void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory, bool achievable) = 0;

    /**
     * Get the current time.
     * @return the current time in seconds
     */
    virtual double getTime() const = 0;
};

#endif //SRC_TRAJECTORYDISPLAYER_H
