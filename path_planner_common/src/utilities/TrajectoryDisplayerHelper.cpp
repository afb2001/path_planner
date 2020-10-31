#include <path_planner_common/TrajectoryDisplayerHelper.h>
#include <geographic_msgs/GeoPoint.h>
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geographic_visualization_msgs/GeoVizPointList.h"
#include "project11_transformations/local_services.h"
#include <path_planner_common/StateMsg.h>
#include <path_planner_common/State.h>

#include <vector>

TrajectoryDisplayerHelper::TrajectoryDisplayerHelper(ros::NodeHandle& nodeHandle, ros::Publisher* displayPub): m_transformations(new project11::Transformations(nodeHandle))
{
    m_display_pub = displayPub;
}

TrajectoryDisplayerHelper::TrajectoryDisplayerHelper(): m_transformations(nullptr) {
    m_display_pub = nullptr;
}

void TrajectoryDisplayerHelper::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) {
    displayTrajectory(trajectory, plannerTrajectory, true);
}

void TrajectoryDisplayerHelper::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory,
                                                  bool achievable) {
    if (!m_display_pub) throw std::runtime_error("Trajectory displayer not properly initialized");
    geographic_visualization_msgs::GeoVizPointList displayPoints;
    displayPoints.color.b = 1;
    if (!plannerTrajectory) {
        // controller trajectory
        displayPoints.color.a = 0.8;
        displayPoints.size = 10;
        if (achievable) {
            displayPoints.color.g = 1;
        } else {
            displayPoints.color.b = 0;
            displayPoints.color.r = 1;
        }
    } else {
        // planner trajectory
        displayPoints.color.a = 1;
        displayPoints.size = 3.0;
        if (!achievable) {
            // dangerous, so color it red
            displayPoints.color.b = 0;
            displayPoints.color.r = 1;
        }
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

double TrajectoryDisplayerHelper::getTime() const {
    return ((double)ros::Time::now().toNSec()) / 1e9;
}

geographic_msgs::GeoPoint TrajectoryDisplayerHelper::convertToLatLong(const State& state) {
    if (!m_display_pub || !m_transformations) throw std::runtime_error("Trajectory displayer not properly initialized");
    
    geometry_msgs::Point point;
    
    point.x = state.x();
    point.y = state.y();
    
    return m_transformations->map_to_wgs84(point);
}

path_planner_common::StateMsg TrajectoryDisplayerHelper::convertToStateMsg(const State& state) {
    if (!m_display_pub) throw std::runtime_error("Trajectory displayer not properly initialized");
    path_planner_common::StateMsg stateMsg;
    stateMsg.x = state.x();
    stateMsg.y = state.y();
    stateMsg.heading = state.heading();
    stateMsg.speed = state.speed();
    stateMsg.time = state.time();
    return stateMsg;
}

State TrajectoryDisplayerHelper::convertToStateFromMsg(const path_planner_common::StateMsg& stateMsg) {
    if (!m_display_pub) throw std::runtime_error("Trajectory displayer not properly initialized");
    State state;
    state.x() = stateMsg.x;
    state.y() = stateMsg.y;
    state.heading() = stateMsg.heading;
    state.speed() = stateMsg.speed;
    state.time() = stateMsg.time;
    return state;
}
