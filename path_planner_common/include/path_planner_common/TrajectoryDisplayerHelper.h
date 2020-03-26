#ifndef SRC_TRAJECTORYDISPLAYERUTILITIES_H
#define SRC_TRAJECTORYDISPLAYERUTILITIES_H

#include <ros/ros.h>
#include <geographic_msgs/GeoPoint.h>
#include "State.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geographic_visualization_msgs/GeoVizPointList.h"
#include "project11_transformations/MapToLatLong.h"
#include <path_planner_common/StateMsg.h>

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
     * constructor if you're going to use it.
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

    geographic_msgs::GeoPoint convertToLatLong(const State& state);

    path_planner_common::StateMsg getStateMsg(const State& state);

    State getState(const path_planner_common::StateMsg& stateMsg);
protected:
    ros::Publisher* m_display_pub;
    ros::ServiceClient m_map_to_lat_long_client;
};

#endif //SRC_TRAJECTORYDISPLAYERUTILITIES_H
