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
#include "NodeBase.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCInconsistentNamingInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

/**
 * Node to act as interface between ROS and path planning system.
 */
class ControllerTest : public NodeBase
{
public:
    explicit ControllerTest(std::string name):
            NodeBase(name)
    {}

    void goalCallback() override
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

    void preemptCallback() override
    {
        std::cerr << "Canceling controller test run" << std::endl;
        m_action_server.setPreempted();
        m_Preempted = true;

        // Should the executive stop now? Probably?
        publishControllerMessage("stop sending controls");
        clearDisplay();
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg) override
    {
        if (m_ActionDone) allDone();
    }

    void publishTrajectory(std::vector<State> trajectory)
    {
        path_planner::Trajectory reference;
        for (State s : trajectory) {
            reference.states.push_back(getStateMsg(s));
        }
        reference.trajectoryNumber = ++m_TrajectoryCount;
        mpc::UpdateReferenceTrajectoryRequest req;
        mpc::UpdateReferenceTrajectoryResponse res;
        req.trajectory = reference;
        if (m_update_reference_trajectory_client.call(req, res)) {
            // success
        } else {
            std::cerr << "Controller failed to send state to test node" << std::endl;
        }
    }

    void allDone() override
    {
        m_ActionDone = false;
        path_planner::path_plannerResult result;
        m_action_server.setSucceeded(result);
        std::cerr << "The times in the trajectory have now all passed. Setting the succeeded bit in the action server." << std::endl;

        publishControllerMessage("stop sending controls");
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
    std::vector<State> m_Trajectory;

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
