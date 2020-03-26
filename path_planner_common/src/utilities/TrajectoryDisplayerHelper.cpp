#include <path_planner_common/TrajectoryDisplayerHelper.h>


TrajectoryDisplayerHelper::TrajectoryDisplayerHelper(ros::NodeHandle& nodeHandle, ros::Publisher* displayPub) {
    m_map_to_lat_long_client = nodeHandle.serviceClient<project11_transformations::MapToLatLong>("map_to_wgs84");
    m_display_pub = displayPub;
}

TrajectoryDisplayerHelper::TrajectoryDisplayerHelper() {
    m_display_pub = nullptr;
}

void TrajectoryDisplayerHelper::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory) {
    displayTrajectory(trajectory, plannerTrajectory, true);
//        for (const auto& s : trajectory) std::cerr << s.toString() << std::endl; // print trajectory
}

void TrajectoryDisplayerHelper::displayTrajectory(const std::vector<State>& trajectory, bool plannerTrajectory,
                                                  bool achievable) {
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

double TrajectoryDisplayerHelper::getTime() const {
    return ((double)ros::Time::now().toNSec()) / 1e9;
}

geographic_msgs::GeoPoint TrajectoryDisplayerHelper::convertToLatLong(const State& state) {
    assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
    project11_transformations::MapToLatLong::Request request;
    project11_transformations::MapToLatLong::Response response;
    request.map.point.x = state.x();
    request.map.point.y = state.y();
    m_map_to_lat_long_client.call(request, response);
    return response.wgs84.position;
}

path_planner_common::StateMsg TrajectoryDisplayerHelper::getStateMsg(const State& state) {
    assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
    path_planner_common::StateMsg stateMsg;
    stateMsg.x = state.x();
    stateMsg.y = state.y();
    stateMsg.heading = state.heading();
    stateMsg.speed = state.speed();
    stateMsg.time = state.time();
    return stateMsg;
}

State TrajectoryDisplayerHelper::getState(const path_planner_common::StateMsg& stateMsg) {
    assert(m_display_pub != nullptr && "Trajectory displayer not properly initialized");
    State state;
    state.x() = stateMsg.x;
    state.y() = stateMsg.y;
    state.heading() = stateMsg.heading;
    state.speed() = stateMsg.speed;
    state.time() = stateMsg.time;
    return state;
}
