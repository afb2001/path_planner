#ifndef SRC_TRAJECTORYDISPLAYER_H
#define SRC_TRAJECTORYDISPLAYER_H

#include <ros/ros.h>
#include <geographic_msgs/GeoPoint.h>
#include "State.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geographic_visualization_msgs/GeoVizPointList.h"
#include "project11_transformations/MapToLatLong.h"

/**
 * Utility class which holds the code for displaying trajectories to /project11/display. It has a node handle, so
 * subclasses need not re-declare one.
 *
 * TODO! -- should change this design to composition, rather than inheritance
 */
class TrajectoryDisplayer {
public:
    /**
     * Initialize a TrajectoryDisplayer. This sets up a ROS service client for map_to_wgs84 and a publisher for
     * /project11/display.
     */
    TrajectoryDisplayer() {
        m_map_to_lat_long_client = m_node_handle.serviceClient<project11_transformations::MapToLatLong>("map_to_wgs84");
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",1);
    }

    /**
     * Display a trajectory to /project11/display. Map coordinates are converted into lat/long with the map_to_wgs84
     * service.
     * @param trajectory the trajectory to display
     * @param plannerTrajectory flag determining color and id
     */
    void displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) {
        geographic_visualization_msgs::GeoVizPointList displayPoints;
        displayPoints.color.b = 1;
        if (!plannerTrajectory) {
//            std::cerr << "Displaying controller trajectory of length " << trajectory.size();
//            if (!trajectory.empty()) {
//                std::cerr << " starting at " << trajectory.front().toString() << std::endl;
//            } else {
//                std::cerr << std::endl;
//            }

            displayPoints.color.g = 1;
            displayPoints.color.a = 0.8;
            displayPoints.size = 10;
        } else {
            displayPoints.color.a = 1;
            displayPoints.size = 3.0;
        }
//        std::cerr << "Displaying controller trajectory: " << std::endl;
        for (const State& s : trajectory) {
//            std::cerr << s.toString() << std::endl;
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
        m_display_pub.publish(geoVizItem);
    }

    /**
     * Get the current time.
     * @return the current time in seconds
     */
    virtual double getTime() const {
        return ((double)ros::Time::now().toNSec()) / 1e9;
    };

protected:
    geographic_msgs::GeoPoint convertToLatLong(const State& state) {
        project11_transformations::MapToLatLong::Request request;
        project11_transformations::MapToLatLong::Response response;
        request.map.point.x = state.x;
        request.map.point.y = state.y;
        m_map_to_lat_long_client.call(request, response);
        return response.wgs84.position;
    }

    path_planner::StateMsg getStateMsg(const State& state) {
        path_planner::StateMsg stateMsg;
        stateMsg.x = state.x;
        stateMsg.y = state.y;
        stateMsg.heading = state.heading;
        stateMsg.speed = state.speed;
        stateMsg.time = state.time;
        return stateMsg;
    }

    State getState(const path_planner::StateMsg& stateMsg) {
        State state;
        state.x = stateMsg.x;
        state.y = stateMsg.y;
        state.heading = stateMsg.heading;
        state.speed = stateMsg.speed;
        state.time = stateMsg.time;
        return state;
    }

    ros::NodeHandle m_node_handle;
    ros::Publisher m_display_pub;
    ros::ServiceClient m_map_to_lat_long_client;
};

#endif //SRC_TRAJECTORYDISPLAYER_H
