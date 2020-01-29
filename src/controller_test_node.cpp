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

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCInconsistentNamingInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

/**
 * Node to act as interface between ROS and path planning system.
 */
class ControllerTest : public TrajectoryDisplayer
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

        std::cerr << "Received " << goal->path.poses.size() - 1 << " survey line(s)" << std::endl;

        m_Trajectory.clear();

        double time = getTime() + 10;

        for (unsigned long i = 0; i < goal->path.poses.size() - 1; i ++) {
            auto startPoint = convertToMap(goal->path.poses[i]);
            auto endPoint = convertToMap(goal->path.poses[i + 1]);
            State start(startPoint.x, startPoint.y, 0, c_MaxSpeed, time);
            State end(endPoint.x, endPoint.y, 0, 0, 0);
            start.setHeadingTowards(end);
            std::cerr << "Adding line between\n" << start.toString() << " and\n" << end.toString() << std::endl;
            State current = start;
            auto d = start.distanceTo(end);
            for (int j = 0; j < d; j++){ // 2 m/s updated every half second is 1m of distance each iteration
                m_Trajectory.push_back(current);
                current.setEstimate(0.5 * j, start);
            }
            m_Trajectory.push_back(current);
            time = current.time;
        }

        std::cerr << "Publishing trajectory of length " << m_Trajectory.size() << " to controller" << std::endl;

        displayTrajectory(m_Trajectory, true);
        publishTrajectory(m_Trajectory);

        auto t = async(std::launch::async, [&]{
            auto t = getTime();
//            cerr << "Starting display loop at time " << t << endl;
            int i = 0;
            while (i < m_Trajectory.size()) {
                t = getTime();
                while (i < m_Trajectory.size() && m_Trajectory[i].time < t) i++;
                if (i > 0) displayPlannerStart(m_Trajectory[i - 1]);
                sleep(1);
                if (m_Preempted) {
                    break;
                }
            }
            if (m_Preempted) {
                m_Preempted = false;
            } else {
                m_ActionDone = true;
            }
            clearDisplay();
        });
    }

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

    void preemptCallback()
    {
        std::cerr << "Canceling controller test run" << std::endl;
        m_action_server.setPreempted();
        m_Preempted = true;

        // Should the executive stop now? Probably?
        publishControllerMessage("stop sending controls");
        clearDisplay();
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg)
    {
        if (m_ActionDone) allDone();
    }

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    void publishTrajectory(std::vector<State> trajectory)
    {
        path_planner::Trajectory reference;
        for (State s : trajectory) {
            reference.states.push_back(getStateMsg(s));
        }
        reference.trajectoryNumber = ++m_TrajectoryCount;
        m_reference_trajectory_pub.publish(reference);
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
        std::cerr << "EstimateState service call failed" << std::endl;
        return State(-1);
    }

    void allDone()
    {
        m_ActionDone = false;
        path_planner::path_plannerResult result;
        m_action_server.setSucceeded(result);
        std::cerr << "The times in the trajectory have now all passed. Setting the succeeded bit in the action server." << std::endl;

        publishControllerMessage("stop sending controls");
    }

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
        geoVizItem.id = "reference_tracker";
        m_display_pub.publish(geoVizItem);
    }

    void displayDot(const State& s) {
        std::cerr << "Displaying dot at state " << s.toString() << std::endl;
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geoVizItem.id = "reference_tracker";
        geographic_visualization_msgs::GeoVizPointList displayPoints;
        displayPoints.color.r = 1;
        displayPoints.color.g = 1;
        displayPoints.color.b = 1;
        displayPoints.color.a = 0.5;
        displayPoints.size = 8;
        displayPoints.points.push_back(convertToLatLong(s));
        geoVizItem.lines.push_back(displayPoints);
        m_display_pub.publish(geoVizItem);
    }

private:
//    ros::NodeHandle m_node_handle;
//    ros::Publisher m_display_pub;
//    ros::ServiceClient m_map_to_lat_long_client;
    actionlib::SimpleActionServer<path_planner::path_plannerAction> m_action_server;
    bool m_ActionDone = false, m_Preempted = false;

    std::vector<State> m_Trajectory;

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

    static constexpr double c_MaxSpeed = 2.0;
};

int main(int argc, char **argv)
{
    std::cerr << "Starting controller test node" << std::endl;
    ros::init(argc, argv, "controller_test");
    ControllerTest pp("path_planner_action");
    ros::spin();

    return 0;
}

#pragma clang diagnostic pop
