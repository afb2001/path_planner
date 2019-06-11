#include <utility>

#include <utility>

#include "ros/ros.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPath.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "marine_msgs/Helm.h"
#include "marine_msgs/Contact.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include "path_planner/path_plannerAction.h"
#include "actionlib/server/simple_action_server.h"
#include "path_planner/Trajectory.h"
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <project11_transformations/LatLongToMap.h>
#include <thread>
#include <signal.h>
#include <geometry_msgs/PoseStamped.h>
#include <project11_transformations/MapToLatLong.h>
#include <mpc/EstimateState.h>
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geographic_visualization_msgs/GeoVizPointList.h"
#include "path_planner/TrajectoryDisplayer.h"

#include "executive/executive.h"
#include "trajectory_publisher.h"


/**
 * Node to act as interface between ROS and path planning system.
 * For now that system includes a controller, which conveys controls
 * to this node through the ControlReceiver interface.
 */
class PathPlanner: public TrajectoryDisplayer, public TrajectoryPublisher
{
public:
    explicit PathPlanner(std::string name): TrajectoryDisplayer(),
    m_action_server(m_node_handle, std::move(name), false)
{
    m_current_speed = 3.0;
    m_current_heading = 0;

    m_lat_long_to_map_client = m_node_handle.serviceClient<project11_transformations::LatLongToMap>("wgs84_to_map");
    m_estimate_state_client = m_node_handle.serviceClient<mpc::EstimateState>("/mpc/estimate_state");

    m_controller_msgs_pub = m_node_handle.advertise<std_msgs::String>("/controller_msgs",1);
    m_reference_trajectory_pub = m_node_handle.advertise<path_planner::Trajectory>("/reference_trajectory",1);


    m_position_sub = m_node_handle.subscribe("/position_map", 10, &PathPlanner::positionCallback, this);
    m_heading_sub = m_node_handle.subscribe("/heading", 10, &PathPlanner::headingCallback, this);
    m_speed_sub = m_node_handle.subscribe("/sog", 10, &PathPlanner::speedCallback, this);
    m_contact_sub = m_node_handle.subscribe("/contact", 10, &PathPlanner::contactCallback, this);

    m_action_server.registerGoalCallback(boost::bind(&PathPlanner::goalCallback, this));
    m_action_server.registerPreemptCallback(boost::bind(&PathPlanner::preemptCallback, this));
    m_action_server.start();

    m_Executive = new Executive(this);
}

    ~PathPlanner() final
    {
        publishControllerMessage("stop running");
        delete m_Executive;
        std::cerr << strerror(errno) << endl;
    }

    // This is really only designed to work once right now
    void goalCallback()
    {
        std::cerr << "Grabbing goal...";
        auto goal = m_action_server.acceptNewGoal();
        std::cerr << " done." << std::endl;

        // make sure controller is up
        publishControllerMessage("start running");


        // if executive is already running, shut it down
        m_Executive->pause();
//        return;
        publishControllerMessage("start sending controls");

        std::cerr << "Touching goal for the first time...";
        m_current_speed = goal->speed;
        std::cerr << " done." << std::endl;

        std::vector<std::pair<double, double>> currentPath;

        std::cerr << "Received " << goal->path.poses.size() << " points to cover" << std::endl;

        // interpolate points to cover from segments
        for (int i = 0; i + 1 < goal->path.poses.size(); i++)
        {
            // assume points represent track-line pairs
            geographic_msgs::GeoPoseStamped startPose = goal->path.poses[i];
            geographic_msgs::GeoPoseStamped endPose = goal->path.poses[i+1];
            geometry_msgs::PointStamped::_point_type start;
            geometry_msgs::PointStamped::_point_type end;

            project11_transformations::LatLongToMap::Request request;
            project11_transformations::LatLongToMap::Response response;

            // send to LatLongToMap
            request.wgs84.position = startPose.pose.position;
            if (m_lat_long_to_map_client.call(request, response)) {
//                start = m_lat_long_to_map_service.response.map.point;
                start = response.map.point;
            }
            currentPath.emplace_back(start.x, start.y);
            request.wgs84.position = endPose.pose.position;
            if (m_lat_long_to_map_client.call(request, response)) {
                end = response.map.point;
            }

            // interpolate and add to interpolatedPoints
            double n, d, dx, dy;
            // total distance
            dx = end.x - start.x; dy = end.y - start.y;
            d = sqrt(dx * dx + dy * dy);
            // number of points
            n = ceil(d / c_max_goal_distance);
            // distance between each point
            dx = dx / n; dy = dy / n;
            for (int j = 1; j < n - 1; j++) {
                currentPath.emplace_back(start.x + (j*dx), start.y + (j*dy));
            }

            currentPath.emplace_back(end.x, end.y);
        }

        for (auto p : currentPath) {
            m_Executive->addToCover((int)p.first, (int)p.second);
        }

        // start planner
        m_Executive->startPlanner("NOFILE");
    }

    void preemptCallback()
    {
        m_action_server.setPreempted();

        // Should the executive stop now? Probably?
        m_Executive->pause();
        publishControllerMessage("stop sending controls");
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg)
    {
        m_Executive->updateCovered(
                inmsg->pose.position.x,
                inmsg->pose.position.y,
                m_current_speed,
                m_current_heading,
                inmsg->header.stamp.toNSec() / 1.0e9);
    }

    void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
    {
        m_current_heading = inmsg->orientation.heading * M_PI / 180.0;
    }

    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
    {
        m_current_speed = inmsg->twist.linear.x; // this will change once /sog is a vector
    }

    void contactCallback(const marine_msgs::Contact::ConstPtr& inmsg)
    {
        State obstacle;

        project11_transformations::LatLongToMap::Request request;
        project11_transformations::LatLongToMap::Response response;

        request.wgs84.position = inmsg->position;
        if (m_lat_long_to_map_client.call(request, response)) {
            obstacle.x = response.map.point.x;
            obstacle.y = response.map.point.y;
        } else {
            std::cerr << "Error: LatLongToMap failed" << endl;
        }

        obstacle.heading = inmsg->cog;
        obstacle.speed = inmsg->sog;

        obstacle.time = inmsg->header.stamp.toNSec() / 1.0e9;

        m_Executive->updateDyamicObstacle(inmsg->mmsi, obstacle);
    }

    void publishTrajectory(vector<State> trajectory) final
    {
        path_planner::Trajectory reference;
        for (State s : trajectory) {
            // explicit conversion to make this cleaner
            reference.states.push_back((path_planner::StateMsg)s);
        }
        m_reference_trajectory_pub.publish(reference);
    }

    void displayTrajectory(vector<State> trajectory, bool plannerTrajectory) override
    {
        TrajectoryDisplayer::displayTrajectory(trajectory, plannerTrajectory);
    }

    State getEstimatedState(double desiredTime) final
    {
        mpc::EstimateStateRequest req;
        mpc::EstimateStateResponse res;
        req.desiredTime = desiredTime;
        if (m_estimate_state_client.call(req, res)) {
            return State(res.state);
        }
        cerr << "EstimateState service call failed" << endl;
        return State(-1);
    }

    void allDone() final
    {
        std::cerr << "Planner appears to have finished" << std::endl;
        path_planner::path_plannerResult result;
        m_action_server.setSucceeded(result);

        // publish empty trajectory to clear the display
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geoVizItem.id = "planner_trajectory";
        m_display_pub.publish(geoVizItem);
        geoVizItem.id = "controller_trajectory";
        m_display_pub.publish(geoVizItem);

        publishControllerMessage("stop sending controls");
    }

    void publishControllerMessage(string m)
    {
        std_msgs::String msg;
        msg.data = std::move(m);
        m_controller_msgs_pub.publish(msg);
    }

private:
//    ros::NodeHandle m_node_handle;
    actionlib::SimpleActionServer<path_planner::path_plannerAction> m_action_server;

    // Since speed and heading are updated through different topics than position,
    // but we need them for state updates to the executive, keep the latest of each
    // to send when we get a new position
    double m_current_speed;
    double m_current_heading;

    ros::Publisher m_controller_msgs_pub;
    ros::Publisher m_reference_trajectory_pub;
//    ros::Publisher m_display_pub;

    ros::Subscriber m_position_sub;
    ros::Subscriber m_heading_sub;
    ros::Subscriber m_speed_sub;
    ros::Subscriber m_contact_sub;

    ros::ServiceClient m_lat_long_to_map_client;
//    ros::ServiceClient m_map_to_lat_long_client;
    ros::ServiceClient m_estimate_state_client;

    // handle on Executive
    Executive* m_Executive;
    // constant for linear interpolation of points to cover
    const double c_max_goal_distance = 10;
};

int main(int argc, char **argv)
{
    std::cerr << "Starting planner node" << endl;
    ros::init(argc, argv, "path_planner");
    PathPlanner pp("path_planner_action");
    ros::spin();

    return 0;
}
