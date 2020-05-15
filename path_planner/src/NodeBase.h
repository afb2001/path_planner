#ifndef SRC_NODEBASE_H
#define SRC_NODEBASE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "path_planner/path_plannerAction.h"
#include "actionlib/server/simple_action_server.h"
#include <project11_transformations/LatLongToMap.h>
#include <geometry_msgs/PoseStamped.h>
#include "trajectory_publisher.h"
#include "path_planner_common/TrajectoryDisplayerHelper.h"
#include <path_planner_common/UpdateReferenceTrajectory.h>
#include <path_planner_common/DubinsPlan.h>
#include <path_planner_common/DubinsPath.h>
#include <geographic_visualization_msgs/GeoVizItem.h>
#include <project11_transformations/local_services.h>

/**
 * Base class for nodes related to the path planner. Holds some shared code and does some shared setup.
 */
class NodeBase
{
public:
    explicit NodeBase(std::string name):
            m_action_server(m_node_handle, std::move(name), false), m_CoordinateConverter(m_node_handle)
    {
        m_current_speed = 0.01;
        m_current_heading = 0;

        m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",1);

        m_controller_msgs_pub = m_node_handle.advertise<std_msgs::String>("/controller_msgs",1);
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("/project11/display",1);

        m_update_reference_trajectory_client = m_node_handle.serviceClient<path_planner_common::UpdateReferenceTrajectory>("/mpc/update_reference_trajectory");

        m_position_sub = m_node_handle.subscribe("/position_map", 10, &NodeBase::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading", 10, &NodeBase::headingCallback, this);
        m_speed_sub = m_node_handle.subscribe("/sog", 10, &NodeBase::speedCallback, this);
        m_piloting_mode_sub = m_node_handle.subscribe("/project11/piloting_mode", 10, &NodeBase::pilotingModeCallback, this);

        m_action_server.registerGoalCallback(boost::bind(&NodeBase::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&NodeBase::preemptCallback, this));
        m_action_server.start();

        m_TrajectoryDisplayer = TrajectoryDisplayerHelper(m_node_handle, &m_display_pub);
    }

    ~NodeBase()
    {
        publishControllerMessage("terminate");
    }

    /**
     * Goal callback for action server.
     */
    virtual void goalCallback() = 0;

    /**
     * Preempt callback for action server.
     */
    virtual void preemptCallback() = 0;

    /**
     * Callback to update vehicle position.
     * @param inmsg
     */
    virtual void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg) = 0;

    /**
     * Callback to update vehicle heading.
     * @param inmsg
     */
    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    /**
     * Callback to update vehicle speed.
     * @param inmsg
     */
    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    /**
     * Callback to update piloting mode.
     * @param inmsg
     */
    virtual void pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg) = 0;

    /**
     * What to do when the planner finishes.
     */
    virtual void allDone() = 0;

    /**
     * Publish a message to the controller.
     * @param m
     */
    void publishControllerMessage(std::string m)
    {
        std_msgs::String msg;
        msg.data = std::move(m);
        m_controller_msgs_pub.publish(msg);
    }

    /**
     * Display the contents of a ribbon manager.
     * @param ribbonManager
     */
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

    /**
     * Display the start state for the current planning iteration.
     * @param state
     */
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

    /**
     * Clear the display. Not sure why this doesn't work.
     */
    void clearDisplay() {
        displayRibbons(RibbonManager());
        m_TrajectoryDisplayer.displayTrajectory(std::vector<State>(), true);
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geoVizItem.id = "planner_start";
        m_display_pub.publish(geoVizItem);
        geoVizItem.id = "reference_tracker";
        m_display_pub.publish(geoVizItem);
    }

    /**
     * Utility to get the time.
     * @return
     */
    double getTime() const {
        return m_TrajectoryDisplayer.getTime();
    }

    /**
     * Convert a state (local map coordinates) to a GeoPoint (LatLong).
     * @param state
     * @return
     */
    geographic_msgs::GeoPoint convertToLatLong(const State& state) {
        return m_TrajectoryDisplayer.convertToLatLong(state);
    }

    /**
     * Convert an internal Dubins plan to a ROS message.
     * @param plan
     * @return
     */
    static path_planner_common::Plan convertToPlanMsg(const DubinsPlan& plan) {
        path_planner_common::Plan planMsg;
        for (const auto& d : plan.get()) {
            path_planner_common::DubinsPath path;
            auto p = d.unwrap();
            path.initial_x = p.qi[0];
            path.initial_y = p.qi[1];
            path.initial_yaw = p.qi[2];
            path.length0 = p.param[0];
            path.length1 = p.param[1];
            path.length2 = p.param[2];
            path.type = p.type;
            path.rho = d.getRho();
            path.speed = d.getSpeed();
            path.start_time = d.getStartTime();
            planMsg.paths.push_back(path);
        }
        planMsg.endtime = plan.getEndTime();
        return planMsg;
    }

    /**
     * Update the controller's reference trajectory and return the state it provides.
     * @param plan
     * @return
     */
    State publishPlan(const DubinsPlan& plan) {
        path_planner_common::UpdateReferenceTrajectoryRequest req;
        path_planner_common::UpdateReferenceTrajectoryResponse res;
        req.plan = convertToPlanMsg(plan);
        if (m_update_reference_trajectory_client.call(req, res)) {
            auto s = m_TrajectoryDisplayer.convertToStateFromMsg(res.state);
            displayPlannerStart(s);
            return s;
        } else {
            return State();
        }
    }

protected:
    ros::NodeHandle m_node_handle;

    TrajectoryDisplayerHelper m_TrajectoryDisplayer;

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

    ros::ServiceClient m_update_reference_trajectory_client;

    project11::Transformations m_CoordinateConverter;
};


#endif //SRC_NODEBASE_H
