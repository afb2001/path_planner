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
#include <path_planner_common/Stats.h>
#include <path_planner_common/TaskLevelStats.h>

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

        m_stats_pub = m_node_handle.advertise<path_planner_common::Stats>("/path_planner/stats", 1);
        m_task_level_stats_pub = m_node_handle.advertise<path_planner_common::TaskLevelStats>("/path_planner/task_level_stats", 1);

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

        for (int i = 0; i + 1 < goal->path.poses.size(); i+= 1) {
            // assume points represent track-line pairs, and that each line gets two points (they don't share points)
            // this will skip every other line the way the mission manager currently sends track lines, but allows for
            // lines to not be connected
            geometry_msgs::PointStamped::_point_type start = m_CoordinateConverter.wgs84_to_map(goal->path.poses[i].pose.position);
            geometry_msgs::PointStamped::_point_type end = m_CoordinateConverter.wgs84_to_map(goal->path.poses[i + 1].pose.position);

            m_Executive->addRibbon(start.x, start.y, end.x, end.y);
        }

        // set the goal speed as the max speed
        path_planner::path_plannerConfig config;
        config.max_speed = goal->speed;

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

        // TODO! -- this assumes position is in the center of the contact. The message allows for that not to be the
        //  case but it always will be for my tests. To fix it you'd just need to shift the position over a bit based
        //  on the asymmetry in the measurements, but I'm pressed for time.

        obstacle.x() = point.x;
        obstacle.y() = point.y;

        obstacle.heading() = inmsg->cog;
        obstacle.speed() = inmsg->sog;

        obstacle.time() = inmsg->header.stamp.toNSec() / 1.0e9;

        // get dimensions with some buffer
        auto width = inmsg->dimension_to_port + inmsg->dimension_to_stbd;
        auto length = inmsg->dimension_to_bow + inmsg->dimension_to_stern;

        if (width <= 5) width = 10;
        if (length <= 10) length = 30;

        m_Executive->updateDynamicObstacle(inmsg->mmsi, obstacle, width, length);
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

    void displayTrajectory(std::vector<State> trajectory, bool plannerTrajectory, bool dangerous) override
    {
        m_TrajectoryDisplayer.displayTrajectory(trajectory, plannerTrajectory, !dangerous);
    }

    void allDone() final
    {
//        std::cerr << "Planner appears to have finished" << std::endl;
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
                                      config.max_speed, config.slow_speed, config.line_width, config.branching_factor,
                                      config.heuristic,
                                      config.time_horizon, config.time_minimum,
                                      config.collision_checking_increment,
                                      config.initial_samples,
                                      config.use_brown_paths,
                                      config.dynamic_obstacles == 1, config.ignore_dynamic_obstacles,
                                      config.use_potential_field_planner);
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

    void displayDynamicObstacle(double x, double y, double yaw, double width, double length, uint32_t id) override {
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        std::stringstream fmt; fmt << "obstacle_" << id;
        geoVizItem.id = fmt.str();
        geographic_visualization_msgs::GeoVizPolygon polygon;
        geographic_visualization_msgs::GeoVizSimplePolygon simplePolygon;
        auto length_x = length / 2 * cos(yaw);
        auto length_y = length / 2 * sin(yaw);
        auto width_x = width / 2 * cos(yaw - M_PI_2);
        auto width_y = width / 2 * sin(yaw - M_PI_2);
        auto bow_port = convertToLatLong(State(x + length_x + width_x, y + length_y + width_y, 0, 0, 0));
        auto bow_starboard = convertToLatLong(State(x + length_x - width_x, y + length_y - width_y, 0, 0, 0));
        auto stern_port = convertToLatLong(State(x - length_x + width_x, y - length_y + width_y, 0, 0, 0));
        auto stern_starboard = convertToLatLong(State(x - length_x - width_x, y - length_y - width_y, 0, 0, 0));
        simplePolygon.points.push_back(bow_port); simplePolygon.points.push_back(bow_starboard);
        simplePolygon.points.push_back(stern_starboard); simplePolygon.points.push_back(stern_port);
        polygon.outer = simplePolygon;
        polygon.edge_color.a = 0.2;
        polygon.edge_color.b = 1;
        polygon.fill_color = polygon.edge_color;
        geoVizItem.polygons.push_back(polygon);
        m_display_pub.publish(geoVizItem);
    }

    void displayMap(std::string path) override {
        geographic_visualization_msgs::GeoVizItem geoVizItem;
        geoVizItem.id = "GridWorldMap";

        // publish empty map to hopefully wipe previous one
        if (path.empty()) {
            m_display_pub.publish(geoVizItem);
            return;
        }

        // parse map file
        std::ifstream infile(path);
        std::string line;
        std::vector<std::string> lines;
        int cols = -1, rows = 0;
        double resolution;
        try {
            std::getline(infile, line);
            std::istringstream s(line);
            s >> resolution;
            while (std::getline(infile, line)) {
                if (cols == -1) cols = line.length();
                else if (line.length() < cols) cols = line.length();
                rows++;
                lines.push_back(line);
            }
        } catch (...) {
            std::cerr << "Error loading map for display. Ignoring" << std::endl;
            m_display_pub.publish(geoVizItem);
            return;
        }
        if (lines.empty()) {
            m_display_pub.publish(geoVizItem);
            return;
        }
        std::reverse(lines.begin(), lines.end());

        // add squares of appropriate size at blocked coordinates
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (lines[y][x] == '#') {
//                    std::cerr << "Adding blocked square to the map" << std::endl;
                    geographic_visualization_msgs::GeoVizPolygon polygon;
                    geographic_visualization_msgs::GeoVizSimplePolygon simplePolygon;
                    auto xCoord = x * resolution; auto yCoord = y * resolution;
                    auto bottomLeft = convertToLatLong(State(xCoord, yCoord, 0, 0, 0));
                    auto bottomRight = convertToLatLong(State(xCoord + resolution, yCoord, 0, 0, 0));
                    auto topLeft = convertToLatLong(State(xCoord, yCoord + resolution, 0, 0, 0));
                    auto topRight = convertToLatLong(State(xCoord + resolution, yCoord + resolution, 0, 0, 0));
                    simplePolygon.points.push_back(topRight); simplePolygon.points.push_back(topLeft);
                    simplePolygon.points.push_back(bottomLeft); simplePolygon.points.push_back(bottomRight);
                    polygon.outer = simplePolygon;
                    polygon.edge_color.a = 0.5;
                    polygon.edge_color.r = 0;
                    polygon.edge_color.g = 0;
                    polygon.edge_color.b = 0;
                    polygon.fill_color = polygon.edge_color;
                    geoVizItem.polygons.push_back(polygon);
                }
            }
        }

        // add boundary lines
        auto bottomLeft = convertToLatLong(State(0, 0, 0, 0, 0));
        auto bottomRight = convertToLatLong(State(resolution * lines[0].size(), 0, 0, 0, 0));
        auto topRight = convertToLatLong(State(resolution * lines[0].size(), resolution * lines.size(), 0, 0, 0));
        auto topLeft = convertToLatLong(State(0, resolution * lines.size(), 0, 0, 0));

        geographic_visualization_msgs::GeoVizPointList bottom, right, top, left;
        bottom.points.push_back(bottomLeft); bottom.points.push_back(bottomRight);
        bottom.size = 5;
        bottom.color.r = 0;
        bottom.color.g = 0;
        bottom.color.b = 0;
        bottom.color.a = 1;

        right.points.push_back(topRight); right.points.push_back(bottomRight);
        right.size = 5;
        right.color.r = 0;
        right.color.g = 0;
        right.color.b = 0;
        right.color.a = 1;

        top.points.push_back(topLeft); top.points.push_back(topRight);
        top.size = 5;
        top.color.r = 0;
        top.color.g = 0;
        top.color.b = 0;
        top.color.a = 1;

        left.points.push_back(bottomLeft); left.points.push_back(topLeft);
        left.size = 5;
        left.color.r = 0;
        left.color.g = 0;
        left.color.b = 0;
        left.color.a = 1;

        geoVizItem.lines.push_back(bottom);
        geoVizItem.lines.push_back(right);
        geoVizItem.lines.push_back(top);
        geoVizItem.lines.push_back(left);

        // publish
        m_display_pub.publish(geoVizItem);
    }

    void publishStats(const Planner::Stats& stats, double collisionPenalty, unsigned long cpuTime,
                      bool lastPlanAchievable) override {
//        std::cerr << stats.Samples << " total samples, " << stats.Generated << " generated, "
//                         << stats.Expanded << " expanded in " << stats.Iterations << " iterations. F-value " <<
//                         stats.PlanFValue << std::endl;

        path_planner_common::Stats statsMsg;
        statsMsg.samples = stats.Samples;
        statsMsg.generated = stats.Generated;
        statsMsg.expanded = stats.Expanded;
        statsMsg.iterations = stats.Iterations;
        statsMsg.plan_f_value = stats.PlanFValue;
        statsMsg.plan_collision_penalty = stats.PlanCollisionPenalty;
        statsMsg.plan_time_penalty = stats.PlanTimePenalty;
        statsMsg.plan_h_value = stats.PlanHValue;
        statsMsg.plan_depth = stats.PlanDepth;
        statsMsg.collision_penalty = collisionPenalty;
        statsMsg.cpu_time = cpuTime;
        statsMsg.last_plan_achievable = lastPlanAchievable;
        m_stats_pub.publish(statsMsg);
    }

    void publishTaskLevelStats(double wallClockTime, double cumulativeCollisionPenalty, double cumulativeGValue,
                               double uncoveredLength) override {
        std::cerr << "Finished task in " << std::to_string(wallClockTime) << "s with total collision penalty "
                  << std::to_string(cumulativeCollisionPenalty) << ". That's a score of "
                  << std::to_string(cumulativeGValue) << std::endl;

        path_planner_common::TaskLevelStats stats;
        stats.time = wallClockTime;
        stats.collision_penalty = cumulativeCollisionPenalty;
        stats.score = cumulativeGValue;
        stats.uncovered_length = uncoveredLength;
        m_task_level_stats_pub.publish(stats);
    }

private:
    ros::NodeHandle m_node_handle;

    geographic_msgs::GeoPoint m_origin;

    ros::Subscriber m_contact_sub;
    ros::Subscriber m_origin_sub;

    ros::Publisher m_stats_pub;
    ros::Publisher m_task_level_stats_pub;

    dynamic_reconfigure::Server<path_planner::path_plannerConfig> m_Dynamic_Reconfigure_Server;

    bool m_Paused = false;

    bool m_CurrentGoalIsValid = false;

    // handle on Executive
    Executive* m_Executive;
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
