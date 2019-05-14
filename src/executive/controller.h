//
// Created by alex on 4/24/19.
//

#ifndef SRC_CONTROLLER_H
#define SRC_CONTROLLER_H

#include "../trajectory_publisher.h"
#include <future>

using namespace std;

class Controller
{
public:

    explicit Controller(TrajectoryPublisher* controlReceiver);

    ~Controller();

    /**
     * Update the action request with a new trajectory.
     * @param actionRequest new reference trajectory
     */
    void receiveRequest(State* actionRequest);

    /**
     * Starts a thread for listening to the executive (ActionSender interface)
     * for action requests.
     */
    void startRunning();

    /**
     * Tell the controller to stop issuing controls.
     *
     */
    void terminate();

    /**
     * Set the plan flag.
     * This should mean the reference trajectory is being updated.
     */
    void startSendingControls();

    /**
     * Clear than plan flag.
     * Do this when the reference trajectory is no longer being updated.
     */
    void stopSendingControls();

private:

    TrajectoryPublisher* m_ControlReceiver;

    //model parameter
    double idle_rpm = 0.0;
    double max_rpm = 3200.0;
    double max_rpm_change_rate = 1000.0;
    double prop_ratio = 0.389105058;
    double prop_pitch = 20.0;
    double max_rudder_angle = 30.0;
    double rudder_coefficient = 0.25;
    double rudder_distance = 2.0;
    double mass = 2000.0;
    double max_power = 8948.4;
    double max_speed = 2.75;
    //double probability[4] = {0.5,0.3,0.15,0.05};
    double probability[4] = {0,1,0,0};
    bool debug = true;


    struct pointc
    {
        double x, y, time;
        pointc(double x1, double y1, double time1)
                : x(x1), y(y1), time(time1){};
    };

    double max_prop_speed;
    double max_force;
    double prop_coefficient;
    double drag_coefficient;

    mutex mtx;
    bool running = true;
    bool plan = false; // This is only false until a control request is received, and then true indefinitely.
    State start;
    State actions[4];
//    string path = "";
//    string default_Command = "0,0";
//    int receivePipeFromParent;
    double estimate_effect_speed = 0, estimate_effect_direction = 0;
    double ptime = 0;
    int iteration = 0;
    bool update = true;
    vector<pointc> future;

    promise<void> m_TerminatePromise;

    double radians(double rudder_angle);

//    void readpath(FILE *readstream);

//    void requestAction();

    void estimate(double &rpm, double throttle, double d_time, double &speed, double rudder, double &heading, double &x, double &y);

    void MPC(double &r, double &t);

    void sendAction(std::future<void> terminated);
};


#endif //SRC_CONTROLLER_H
