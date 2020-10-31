#ifndef SRC_TRAJECTORYDISPLAYERUTILITIES_H
#define SRC_TRAJECTORYDISPLAYERUTILITIES_H

#include <geographic_msgs/GeoPoint.h>
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geographic_visualization_msgs/GeoVizPointList.h"
#include "project11_transformations/local_services.h"
#include <path_planner_common/StateMsg.h>
#include "State.h"

#include <vector>

// forward declarations
namespace ros {
    class NodeHandle;
    class Publisher;
}

/**
 * Utility class which holds the code for displaying trajectories to /project11/display.
 */
class TrajectoryDisplayerHelper {
public:
    /**
     * Initialize a TrajectoryDisplayer. This sets up a ROS service client for map_to_wgs84. It expects a publisher to
     * /project11/display.
     */
    TrajectoryDisplayerHelper(ros::NodeHandle& nodeHandle, ros::Publisher* displayPub);

    /**
     * Default constructor to allow for automatic construction. You should assign to your instance using the other
     * constructor if you're going to use it. I don't remember why I needed this but I wouldn't have put it in if I didn't.
     * If you see a way to remove it gracefully go ahead - it would simplify things and be more idiomatic (RAII).
     */
    TrajectoryDisplayerHelper();

    /**
     * Display a trajectory to /project11/display. Map coordinates are converted into lat/long with the map_to_wgs84
     * service.
     * @param trajectory the trajectory to display
     * @param plannerTrajectory flag determining color and id
     */
    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory);

    /**
     * Display a trajectory to /project11/display. Map coordinates are converted into lat/long with the map_to_wgs84
     * service.
     * @param trajectory the trajectory to display
     * @param plannerTrajectory flag determining color and id
     * @param achievable whether this trajectory is achievable
     */
    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory, bool achievable);

    /**
     * Get the current time.
     * @return the current time in seconds
     */
    virtual double getTime() const;;

    /**
     * Converts a state (in map coordinates) to a GeoPoint (in LatLong).
     * @param state
     * @return GeoPoint version of the state (LatLong)
     */
    geographic_msgs::GeoPoint convertToLatLong(const State& state);

    /**
     * Converts a state to a ROS message.
     * @param state
     * @return ROS StateMsg representing the state
     */
    path_planner_common::StateMsg convertToStateMsg(const State& state);

    /**
     * Convert a ROS state message to the internal State type.
     * @param stateMsg
     * @return
     */
    State convertToStateFromMsg(const path_planner_common::StateMsg& stateMsg);

protected:
    ros::Publisher* m_display_pub;
    project11::Transformations* m_transformations;
};

#endif //SRC_TRAJECTORYDISPLAYERUTILITIES_H
