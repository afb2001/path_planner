#include "ros/ros.h"
#include "geographic_msgs/GeoPath.h"
#include "geometry_msgs/TwistStamped.h"
#include "marine_msgs/Contact.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"
#include "path_planner/path_plannerAction.h"
#include <project11_transformations/LatLongToMap.h>
#include <geometry_msgs/PoseStamped.h>
#include "executive/executive.h"
#include "trajectory_publisher.h"
#include "NodeBase.h"
#include <path_planner/path_plannerConfig.h>
#include <dynamic_reconfigure/server.h>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCInconsistentNamingInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

/**
 * Node to act as interface between ROS and path planning system.
 */
class PathPlanner final: public NodeBase, public TrajectoryPublisher
{
public:
    explicit PathPlanner(std::string name): NodeBase(std::move(name))
{
    m_Executive = new Executive(this);

    m_contact_sub = m_node_handle.subscribe("/contact", 10, &PathPlanner::contactCallback, this);
    m_origin_sub = m_node_handle.subscribe("/origin", 1, &PathPlanner::originCallback, this);

    dynamic_reconfigure::Server<path_planner::path_plannerConfig>::CallbackType f;
    f = boost::bind(&PathPlanner::reconfigureCallback, this, _1, _2);
    m_Dynamic_Reconfigure_Server.setCallback(f);
}

    void pilotingModeCallback(const std_msgs::String::ConstPtr& inmsg) override {
        if (inmsg->data == "autonomous") {
            if (m_Paused) {
                // only resume planner if there's an unfinished survey already in the executive
                if (m_CurrentGoalIsValid) {
                    m_Executive->startPlanner();
                }
                m_Paused = false;
            }
        } else {
            // don't need to do anything different on this end when canceling vs pausing
            m_Executive->cancelPlanner();
            m_Paused = true;
        }
    }

    ~PathPlanner() final
    {
        publishControllerMessage("stop running");
        delete m_Executive;
        std::cerr << strerror(errno) << std::endl;
    }

    void goalCallback() override
    {
        // clear paused flag because we're starting fresh
        m_Paused = false;

        auto goal = m_action_server.acceptNewGoal();

        // make sure controller is up
        publishControllerMessage("start running");

        // if executive is already running, shut it down
        m_Executive->cancelPlanner();
//        return;
        publishControllerMessage("start sending controls");

        m_current_speed = goal->speed;

        m_Executive->clearRibbons();

        while (!m_CoordinateConverter.haveOrigin())
            ros::Duration(0.5).sleep();


        std::cerr << "Received " << goal->path.poses.size() << " points to cover" << std::endl;

        for (int i = 0; i + 1 < goal->path.poses.size(); i++) {
            // assume points represent track-line pairs
            geometry_msgs::PointStamped::_point_type start = m_CoordinateConverter.wgs84_to_map(goal->path.poses[i].pose.position);
            geometry_msgs::PointStamped::_point_type end = m_CoordinateConverter.wgs84_to_map(goal->path.poses[i + 1].pose.position);

            m_Executive->addRibbon(start.x, start.y, end.x, end.y);
        }
        // set goal to be valid
        m_CurrentGoalIsValid = true;

        // start planner
        m_Executive->startPlanner();
    }

    void preemptCallback() override
    {
        std::cerr << "Canceling planner" << std::endl;
        m_Preempted = true;
        m_CurrentGoalIsValid = false;
        m_action_server.setPreempted();

        // Should the executive stop now? Probably?
        m_Executive->cancelPlanner();
        publishControllerMessage("terminate");
        clearDisplay();
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &inmsg) override
    {
        m_Executive->updateCovered(
                inmsg->pose.position.x,
                inmsg->pose.position.y,
                m_current_speed,
                m_current_heading,
                m_TrajectoryDisplayer.getTime());
//                inmsg->header.stamp.toNSec() / 1.0e9);

        // need to set succeeded in action server in ROS callback thread for some reason
        if (m_ActionDone) setSucceeded();
    }

    void contactCallback(const marine_msgs::Contact::ConstPtr& inmsg)
    {
        State obstacle;

        auto point = m_CoordinateConverter.wgs84_to_map(inmsg->position);

        obstacle.x() = point.x;
        obstacle.y() = point.y;

        obstacle.heading() = inmsg->cog;
        obstacle.speed() = inmsg->sog;

        obstacle.time() = inmsg->header.stamp.toNSec() / 1.0e9;

        m_Executive->updateDynamicObstacle(inmsg->mmsi, obstacle);
    }

//    State publishTrajectory(std::vector<State> trajectory) final
//    {
//        path_planner::Trajectory reference;
//        for (State s : trajectory) {
//            reference.states.push_back(convertToStateMsg(s));
//        }
//        reference.trajectoryNumber = ++m_TrajectoryCount;
//        mpc::UpdateReferenceTrajectoryRequest req;
//        mpc::UpdateReferenceTrajectoryResponse res;
//        req.trajectory = reference;
//        if (m_update_reference_trajectory_client.call(req, res)) {
//            auto s = convertToStateFromMsg(res.state);
//            displayPlannerStart(s);
//            return s;
//        } else {
//            return State();
//        }
//    }

    void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory) override
    {
        m_TrajectoryDisplayer.displayTrajectory(trajectory, plannerTrajectory);
    }

    void allDone() final
    {
        std::cerr << "Planner appears to have finished" << std::endl;
        m_ActionDone = true;
        m_CurrentGoalIsValid = false;
        publishControllerMessage("terminate");
        clearDisplay();
    }

    void setSucceeded()
    {
        std::cerr << "Setting succeeded bit in action server." << std::endl;
        m_ActionDone = false;
        path_planner::path_plannerResult result;
        m_action_server.setSucceeded(result);
    }

    void reconfigureCallback(path_planner::path_plannerConfig &config, uint32_t level) {
        m_Executive->refreshMap(config.planner_geotiff_map, m_origin.latitude, m_origin.longitude);
        m_Executive->setConfiguration(config.non_coverage_turning_radius, config.coverage_turning_radius,
                                      config.max_speed, config.line_width, config.branching_factor, config.heuristic,
                                      config.time_horizon, config.time_minimum,
                                      config.collision_checking_increment,
                                      config.initial_samples,
                                      config.use_brown_paths);
        m_Executive->setPlannerVisualization(config.dump_visualization, config.visualization_file);
    }

    void originCallback(const geographic_msgs::GeoPointConstPtr& inmsg) {
        m_origin = *inmsg;
    }

    double getTime() const override {
        return m_TrajectoryDisplayer.getTime();
    }

    void displayRibbons(const RibbonManager& ribbonManager) override {

//        std::cerr << ribbonManager.dumpRibbons() << std::endl;

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

    State publishPlan(const DubinsPlan& plan) override {
        return NodeBase::publishPlan(plan);
    }

private:
    ros::NodeHandle m_node_handle; // TODO

    geographic_msgs::GeoPoint m_origin;

    ros::Subscriber m_contact_sub;
    ros::Subscriber m_origin_sub;

    dynamic_reconfigure::Server<path_planner::path_plannerConfig> m_Dynamic_Reconfigure_Server;

    bool m_Paused = false;

    bool m_CurrentGoalIsValid = false;

    // handle on Executive
    Executive* m_Executive;
    // constant for linear interpolation of points to cover
    const double c_max_goal_distance = 10;
};

int main(int argc, char **argv)
{
    std::cerr << "Starting planner node" << std::endl;
    ros::init(argc, argv, "path_planner");
    PathPlanner pp("path_planner_action");
    ros::spin();

    return 0;
}

#pragma clang diagnostic pop
