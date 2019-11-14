#include <utility>
#include "ros/ros.h"
#include "geographic_msgs/GeoPath.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/String.h"
#include "marine_msgs/Contact.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include "path_planner/path_plannerAction.h"
#include "actionlib/server/simple_action_server.h"
#include "path_planner/Trajectory.h"
#include <fstream>
#include <project11_transformations/LatLongToMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <mpc/EstimateStateRequest.h>
#include <mpc/EstimateStateResponse.h>
#include <mpc/EstimateState.h>
#include "executive/executive.h"
#include "trajectory_publisher.h"
#include "path_planner/TrajectoryDisplayer.h"
#include <path_planner/path_plannerConfig.h>
#include <dynamic_reconfigure/server.h>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCInconsistentNamingInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

/**
 * Node to act as interface between ROS and path planning system.
 */
class ControllerTest
{
public:
    explicit ControllerTest(std::string name):
            m_action_server(m_node_handle, std::move(name), false)
    {
        m_current_speed = 3.0;
        m_current_heading = 0;

        m_lat_long_to_map_client = m_node_handle.serviceClient<project11_transformations::LatLongToMap>("wgs84_to_map");
        m_estimate_state_client = m_node_handle.serviceClient<mpc::EstimateState>("/mpc/estimate_state");

        m_controller_msgs_pub = m_node_handle.advertise<std_msgs::String>("/controller_msgs",1);
        m_reference_trajectory_pub = m_node_handle.advertise<path_planner::Trajectory>("/reference_trajectory",1);

        m_position_sub = m_node_handle.subscribe("/position_map", 10, &ControllerTest::positionCallback, this);
        m_heading_sub = m_node_handle.subscribe("/heading", 10, &ControllerTest::headingCallback, this);
        m_speed_sub = m_node_handle.subscribe("/sog", 10, &ControllerTest::speedCallback, this);

        m_action_server.registerGoalCallback(boost::bind(&ControllerTest::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&ControllerTest::preemptCallback, this));
        m_action_server.start();
    }

    ~ControllerTest()
    {
        publishControllerMessage("stop running");
    }

    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();

        // make sure controller is up
        publishControllerMessage("start running");

        publishControllerMessage("start sending controls");

        m_current_speed = goal->speed;

        std::vector<std::pair<double, double>> currentPath;

        std::cerr << "Received " << goal->path.poses.size() << " points to cover" << std::endl;

        // transit mode
        if (goal->path.poses.size() > 2) {
            project11_transformations::LatLongToMap::Request request;
            project11_transformations::LatLongToMap::Response response;
            request.wgs84.position = goal->path.poses.back().pose.position;
            if (m_lat_long_to_map_client.call(request, response)) {
//                m_Executive->addToCover(response.map.point.x, response.map.point.y);
            }
        } else {
            for (int i = 0; i + 1 < goal->path.poses.size(); i++) {
                // assume points represent track-line pairs
                geographic_msgs::GeoPoseStamped startPose = goal->path.poses[i];
                geographic_msgs::GeoPoseStamped endPose = goal->path.poses[i + 1];
                geometry_msgs::PointStamped::_point_type start;
                geometry_msgs::PointStamped::_point_type end;

                project11_transformations::LatLongToMap::Request request;
                project11_transformations::LatLongToMap::Response response;

                // send to LatLongToMap
                request.wgs84.position = startPose.pose.position;
                if (m_lat_long_to_map_client.call(request, response)) {
                    start = response.map.point;
                }
                currentPath.emplace_back(start.x, start.y);
                request.wgs84.position = endPose.pose.position;
                if (m_lat_long_to_map_client.call(request, response)) {
                    end = response.map.point;
                }

//                m_Executive->addRibbon(start.x, start.y, end.x, end.y);
            }
        }
    }

    void preemptCallback()
    {
        cerr << "Canceling controller test run" << endl;
        m_action_server.setPreempted();

        // Should the executive stop now? Probably?
        publishControllerMessage("stop sending controls");
        clearDisplay();
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg)
    {
        // TODO!
    }

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    void publishTrajectory(vector<State> trajectory) final
    {
        path_planner::Trajectory reference;
        for (State s : trajectory) {
            reference.states.push_back(getStateMsg(s));
        }
        reference.trajectoryNumber = ++m_TrajectoryCount;
        m_reference_trajectory_pub.publish(reference);
    }

    void displayTrajectory(vector<State> trajectory, bool plannerTrajectory) override
    {
        TrajectoryDisplayer::displayTrajectory(trajectory, plannerTrajectory);
    }

    State getEstimatedState(double desiredTime)
    {
        mpc::EstimateStateRequest req;
        mpc::EstimateStateResponse res;
        req.desiredTime = desiredTime;
        // loop while we're getting estimates from a stale trajectory
        while (m_estimate_state_client.call(req, res)) {
            if (res.trajectoryNumber == m_TrajectoryCount) {
                auto s = getState(res.state);
                displayPlannerStart(s);
                return s;
            }
        }
        cerr << "EstimateState service call failed" << endl;
        return State(-1);
    }

    void allDone()
    {
        std::cerr << "Planner appears to have finished" << std::endl;
        path_planner::path_plannerResult result;
        m_action_server.setSucceeded(result);

        publishControllerMessage("stop sending controls");
    }

    void publishControllerMessage(string m)
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
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geographic_visualization_msgs::GeoVizPolygon polygon;
        geographic_visualization_msgs::GeoVizSimplePolygon simplePolygon;
        State bow, sternPort, sternStarboard;
        bow.setEstimate(3 / state.speed, state); //set bow 3m ahead of state
        sternPort.setEstimate(-1 / state.speed, state);
        sternStarboard.set(sternPort);
        auto a = state.heading + M_PI_2;
        auto dx = 1.5 * sin(a);
        auto dy = 1.5 * cos(a);
        sternPort.x += dx;
        sternPort.y += dy;
        sternStarboard.x -= dx;
        sternStarboard.y -= dy;
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
        TrajectoryDisplayer::displayTrajectory(std::vector<State>(), true);
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geoVizItem.id = "planner_start";
        m_display_pub.publish(geoVizItem);
    }

private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_display_pub;
    ros::ServiceClient m_map_to_lat_long_client;
    actionlib::SimpleActionServer<path_planner::path_plannerAction> m_action_server;

    // Since speed and heading are updated through different topics than position,
    // but we need them for state updates to the executive, keep the latest of each
    // to send when we get a new position
    double m_current_speed;
    double m_current_heading;

    ros::Publisher m_controller_msgs_pub;
    ros::Publisher m_reference_trajectory_pub;

    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_speed_sub;

    ros::ServiceClient m_lat_long_to_map_client;
    ros::ServiceClient m_estimate_state_client;

    long m_TrajectoryCount = 1;
};

int main(int argc, char **argv)
{
    std::cerr << "Starting controller test node" << endl;
    ros::init(argc, argv, "controller_test");
    ControllerTest pp("path_planner_action");
    ros::spin();

    return 0;
}

#pragma clang diagnostic pop
