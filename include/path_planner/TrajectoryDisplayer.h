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
//            std::cerr << "Displaying controller trajectory of length " << trajectory.size() << std::endl;
            displayPoints.color.g = 1;
        }
        displayPoints.color.a = 1;
        displayPoints.size = 3.0;
        for (State s : trajectory) {
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

protected:
    geographic_msgs::GeoPoint convertToLatLong(State state) {
        project11_transformations::MapToLatLong::Request request;
        project11_transformations::MapToLatLong::Response response;
        request.map.point.x = state.x;
        request.map.point.y = state.y;
        m_map_to_lat_long_client.call(request, response);
        return response.wgs84.position;
    }

    ros::NodeHandle m_node_handle;
    ros::Publisher m_display_pub;
    ros::ServiceClient m_map_to_lat_long_client;
};


#endif //SRC_TRAJECTORYDISPLAYER_H
