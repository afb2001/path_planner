#ifndef SRC_NODEBASE_H
#define SRC_NODEBASE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "path_planner/path_plannerAction.h"
#include "actionlib/server/simple_action_server.h"
#include "path_planner/Trajectory.h"
#include <project11_transformations/LatLongToMap.h>
#include <geometry_msgs/PoseStamped.h>
#include "trajectory_publisher.h"
#include "path_planner/TrajectoryDisplayer.h"
#include <mpc/UpdateReferenceTrajectory.h>
#include <path_planner/Plan.h>
#include <path_planner/DubinsPath.h>

/**
 * Base class for nodes related to the path planner.
 */
class NodeBase
{
public:
    explicit NodeBase(std::string name):
            m_action_server(m_node_handle, std::move(name), false)
    {
        m_current_speed = 0.01;
        m_current_heading = 0;

        m_lat_long_to_map_client = m_node_handle.serviceClient<project11_transformations::LatLongToMap>("wgs84_to_map");
        m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",1);

        m_controller_msgs_pub = m_node_handle.advertise<std_msgs::String>("/controller_msgs",1);
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",1);

        m_update_reference_trajectory_client = m_node_handle.serviceClient<mpc::UpdateReferenceTrajectory>("/mpc/update_reference_trajectory");

        m_position_sub = m_node_handle.subscribe("/position_map", 10, &NodeBase::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading", 10, &NodeBase::headingCallback, this);
        m_speed_sub = m_node_handle.subscribe("/sog", 10, &NodeBase::speedCallback, this);
        m_piloting_mode_sub = m_node_handle.subscribe("/project11/piloting_mode", 10, &NodeBase::pilotingModeCallback, this);

        m_action_server.registerGoalCallback(boost::bind(&NodeBase::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&NodeBase::preemptCallback, this));
        m_action_server.start();

        m_TrajectoryDisplayer = TrajectoryDisplayer(m_node_handle, &m_display_pub);
    }

    ~NodeBase()
    {
        publishControllerMessage("terminate");
    }

    virtual void goalCallback() = 0;

    virtual void preemptCallback() = 0;

    geometry_msgs::Point convertToMap(geographic_msgs::GeoPoseStamped pose) {
        project11_transformations::LatLongToMap::Request request;
        project11_transformations::LatLongToMap::Response response;
        request.wgs84.position = pose.pose.position;
        if (m_lat_long_to_map_client.call(request, response)) {
            return response.map.point;
        } else {
            std::cerr << "LatLongToMap failed" << std::endl;
            return response.map.point;
        }
    }

    virtual void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg) = 0;

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    virtual void pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg) = 0;

    virtual void allDone() = 0;

    void publishControllerMessage(std::string m)
    {
        std_msgs::String msg;
        msg.data = std::move(m);
        m_controller_msgs_pub.publish(msg);
    }

    void displayRibbons(const RibbonManager& ribbonManager) {

        geographic_visualization_msgs::GeoVizItem geoVizItem;

        for (const auto& r : ribbonManager.get()) {
            geographic_visualization_msgs::GeoVizPointList displayPoints;
            displayPoints.color.r = 1;
            displayPoints.color.b = 0.5;
            displayPoints.color.a = 0.6;
            displayPoints.size = 15;
            geographic_msgs::GeoPoint point;
            displayPoints.points.push_back(convertToLatLong(r.startAsState()));
            displayPoints.points.push_back(convertToLatLong(r.endAsState()));
            geoVizItem.lines.push_back(displayPoints);
        }
        geoVizItem.id = "ribbons";
        m_display_pub.publish(geoVizItem);
    }

    void displayPlannerStart(const State& state) {
//        cerr << "Displaying state " << state.toString() << endl;
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geographic_visualization_msgs::GeoVizPolygon polygon;
        geographic_visualization_msgs::GeoVizSimplePolygon simplePolygon;
        State bow, sternPort, sternStarboard;
        bow = state.push(3 / state.speed()); //set bow 3m ahead of state
        sternPort = state.push( -1 / state.speed());
        sternStarboard = sternPort;
        auto a = state.heading() + M_PI_2;
        auto dx = 1.5 * sin(a);
        auto dy = 1.5 * cos(a);
        sternPort.x() += dx;
        sternPort.y() += dy;
        sternStarboard.x() -= dx;
        sternStarboard.y() -= dy;
        simplePolygon.points.push_back(convertToLatLong(bow));
        simplePolygon.points.push_back(convertToLatLong(sternPort));
        simplePolygon.points.push_back(convertToLatLong(sternStarboard));
        polygon.outer = simplePolygon;
        polygon.edge_color.b = 1;
        polygon.edge_color.a = 0.7;
        polygon.fill_color = polygon.edge_color;
        geoVizItem.polygons.push_back(polygon);
        geoVizItem.id = "planner_start";
        m_display_pub.publish(geoVizItem);
    }

    void clearDisplay() {
        displayRibbons(RibbonManager());
        m_TrajectoryDisplayer.displayTrajectory(std::vector<State>(), true);
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geoVizItem.id = "planner_start";
        m_display_pub.publish(geoVizItem);
        geoVizItem.id = "reference_tracker";
        m_display_pub.publish(geoVizItem);
    }

    double getTime() const {
        return m_TrajectoryDisplayer.getTime();
    }

    path_planner::StateMsg getStateMsg(const State& state) {
        return m_TrajectoryDisplayer.getStateMsg(state);
    }

    State getState(const path_planner::StateMsg& stateMsg) {
        return m_TrajectoryDisplayer.getState(stateMsg);
    }

    geographic_msgs::GeoPoint convertToLatLong(const State& state) {
        return m_TrajectoryDisplayer.convertToLatLong(state);
    }

    static path_planner::Plan getPlanMsg(const Plan& plan) {
        path_planner::Plan planMsg;
        for (const auto& d : plan.get()) {
            path_planner::DubinsPath path;
            auto p = d.unwrap();
            path.x = p.qi[0];
            path.y = p.qi[1];
            path.yaw = p.qi[2];
            path.param0 = p.param[0];
            path.param1 = p.param[1];
            path.param2 = p.param[2];
            path.speed = d.getSpeed();
            path.start_time = d.getStartTime();
            path.rho = d.getRho();
            planMsg.paths.push_back(path);
        }
        return planMsg;
    }

    State publishPlan(const Plan& plan) {
        mpc::UpdateReferenceTrajectoryRequest req;
        mpc::UpdateReferenceTrajectoryResponse res;
        req.plan = getPlanMsg(plan);
        if (m_update_reference_trajectory_client.call(req, res)) {
            auto s = getState(res.state);
            displayPlannerStart(s);
            return s;
        } else {
            return State();
        }
    }

protected:
    ros::NodeHandle m_node_handle;

    TrajectoryDisplayer m_TrajectoryDisplayer;

    actionlib::SimpleActionServer<path_planner::path_plannerAction> m_action_server;

    // Apparently the action server is supposed to be manipulated on the ROS thread, so I had to make some flags
    // to get the done/preemption behavior I wanted. It seems like there should be a better way to do this but I don't
    // know what it is.
    bool m_ActionDone = false, m_Preempted = false;

    // Since speed and heading are updated through different topics than position,
    // but we need them for state updates to the executive, keep the latest of each
    // to send when we get a new position
    double m_current_speed;
    double m_current_heading;

    ros::Publisher m_controller_msgs_pub;
    ros::Publisher m_display_pub;

    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_speed_sub;
    ros::Subscriber m_piloting_mode_sub;

    ros::ServiceClient m_lat_long_to_map_client;
    ros::ServiceClient m_update_reference_trajectory_client;

    long m_TrajectoryCount = 1;
};


#endif //SRC_NODEBASE_H
