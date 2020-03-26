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
    TrajectoryDisplayerHelper(ros::NodeHandle& nodeHandle, ros::Publisher* displayPub) {
        m_map_to_lat_long_client = nodeHandle.serviceClient<project11_transformations::MapToLatLong>("map_to_wgs84");
        m_display_pub = displayPub;
    }

    /**
     * Default constructor to allow for automatic construction. You should assign to your instance using the other
     * constructor if you're going to use it.
     */
    TrajectoryDisplayerHelper() {
        m_display_pub = nullptr;
    }

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
    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory, bool achievable) {
        assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
        geographic_visualization_msgs::GeoVizPointList displayPoints;
        displayPoints.color.b = 1;
        if (!plannerTrajectory) {
            displayPoints.color.a = 0.8;
            displayPoints.size = 10;
            if (achievable) {
                displayPoints.color.g = 1;
            } else {
                displayPoints.color.b = 0;
                displayPoints.color.r = 1;
            }
        } else {
            displayPoints.color.a = 1;
            displayPoints.size = 3.0;
        }
        for (const State& s : trajectory) {
            geographic_msgs::GeoPoint point;
            displayPoints.points.push_back(convertToLatLong(s));
        }
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        if (plannerTrajectory) {
            geoVizItem.id = "planner_trajectory";
        } else {
            geoVizItem.id = "controller_trajectory";
        }
        geoVizItem.lines.push_back(displayPoints);
        m_display_pub->publish(geoVizItem);
    }

    /**
     * Get the current time.
     * @return the current time in seconds
     */
    virtual double getTime() const {
        return ((double)ros::Time::now().toNSec()) / 1e9;
    };

    geographic_msgs::GeoPoint convertToLatLong(const State& state) {
        assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
        project11_transformations::MapToLatLong::Request request;
        project11_transformations::MapToLatLong::Response response;
        request.map.point.x = state.x();
        request.map.point.y = state.y();
        m_map_to_lat_long_client.call(request, response);
        return response.wgs84.position;
    }

    path_planner_common::StateMsg getStateMsg(const State& state) {
        assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
        path_planner_common::StateMsg stateMsg;
        stateMsg.x = state.x();
        stateMsg.y = state.y();
        stateMsg.heading = state.heading();
        stateMsg.speed = state.speed();
        stateMsg.time = state.time();
        return stateMsg;
    }

    State getState(const path_planner_common::StateMsg& stateMsg) {
        assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
        State state;
        state.x() = stateMsg.x;
        state.y() = stateMsg.y;
        state.heading() = stateMsg.heading;
        state.speed() = stateMsg.speed;
        state.time() = stateMsg.time;
        return state;
    }
protected:
    ros::Publisher* m_display_pub;
    ros::ServiceClient m_map_to_lat_long_client;
};

#endif //SRC_TRAJECTORYDISPLAYERUTILITIES_H
