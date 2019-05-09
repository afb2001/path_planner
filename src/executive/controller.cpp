#include <iostream>
#include <fstream>
//#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
#include <string.h>
#include <sstream>
#include <cmath>
#include "communication.h"
#include "ObjectPar.h"
#include <cfloat>

#include "controller.h"

using namespace std;

Controller::Controller(ControlReceiver *controlReceiver) {
    m_ControlReceiver = controlReceiver;

    // TODO! -- should allow setting of boat characteristics; used to read from a file I think

    //prepare fore model compute // <-- vintage Chao comment
    max_prop_speed = (max_rpm * prop_ratio) / prop_pitch;
    max_force = max_power / max_speed;
    prop_coefficient = max_force / (max_prop_speed * max_prop_speed - max_speed * max_speed);
    drag_coefficient = max_force / (pow(max_speed, 3));
}

Controller::~Controller() = default;

void Controller::receiveRequest(ObjectPar* actionRequest)
{
    if (running)
    {
        mtx.lock();
        start.set(actionRequest[0]);
        for (int i = 1; i < 5; i++)
        {
            actions[i-1].set(actionRequest[i]);
        }
        mtx.unlock();
        // Don't read a path because nobody's sending one anymore
    }
    delete actionRequest;
}

double Controller::radians(double rudder_angle)
{
    return (rudder_angle * M_PI) / 180;
}

void Controller::estimate(double &rpm, double throttle, double d_time, double &speed, double rudder, double &heading, double &x, double &y)
{
    double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
    double rcr = (target_rpm - rpm) / d_time;

    if (fabs(rcr) > max_rpm_change_rate)
    {
        if (rcr < 0)
            rcr = -max_rpm_change_rate;
        else
            rcr = max_rpm_change_rate;
    }
    rpm += rcr * d_time;

    //seperate this and make a while loop // <-- ??

    double prop_rpm = prop_ratio * rpm;
    double prop_speed = prop_rpm / prop_pitch;
    double rudder_speed = fmax(sqrt(prop_speed), speed);
    double thrust = prop_coefficient * (prop_speed * prop_speed - speed * speed);
    double rudder_angle = rudder * max_rudder_angle;
    double rudder_rads = radians(rudder_angle);
    double thrust_fwd = thrust * cos(rudder_rads);
    double rudder_speed_yaw = rudder_speed * sin(rudder_rads);
    double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

    heading += yaw_rate * d_time;
    heading = fmod(heading, radians(360));
    if (heading < 0)
        heading += radians(360);

    double drag = pow(speed, 3) * drag_coefficient;
    speed += ((thrust_fwd - drag) / mass) * d_time;
    double delta = speed * d_time;
    double deltaEV = estimate_effect_speed * d_time;
    x += delta * sin(heading) + deltaEV * sin(estimate_effect_direction);
    y += delta * cos(heading) + deltaEV * cos(estimate_effect_direction);
}

//take speed & rmp & heading into consideration!!!!! currently do nothing
void Controller::MPC(double &r, double &t)
{
    // grab current starting position and reference trajectory
    mtx.lock();
    ObjectPar startCopy;
    startCopy.set(start);
//    cerr << "Starting MPC from " << startCopy.toString() << endl;
    ObjectPar actionsCopy[4];
//    cerr << "Reference trajectory: " << endl;
    for (int i = 0; i < 4; i++) {
        actionsCopy[i].set(actions[i]);
//        cerr << actionsCopy[i].toString() << endl;
    }
    mtx.unlock();

    double x = startCopy.x, y = startCopy.y, heading = startCopy.heading, speed = startCopy.speed;
    double rpm = idle_rpm + (speed / max_speed) * (max_rpm - idle_rpm);
    double throttle = 0;
    double rudder = -1;
    double duration = 0.05;

    double coefficient = DBL_MAX;

    if (ptime != startCopy.otime && !future.empty() && update)
    {
        if (iteration < 50)
            ++iteration;
        ptime = startCopy.otime;
        double cx = startCopy.x;
        double cy = startCopy.y;
        int index = 0;
        for (int i = 1; i < future.size(); i++)
        {
            if (ptime <= future[i].time)
            {
                index = (fabs(future[i].time - ptime) < fabs(future[i - 1].time - ptime)) ? i : i - 1;
                break;
            }
        }
        double dtime = future[index].time - (future[0].time - 0.05);
        double diffx = (startCopy.x - future[index].x) / dtime;
        double diffy = (startCopy.y - future[index].y) / dtime;
        double deltax = estimate_effect_speed * sin(estimate_effect_direction);
        double deltay = estimate_effect_speed * cos(estimate_effect_direction);
        deltax += diffx / iteration;
        deltay += diffy / iteration;
        estimate_effect_direction = atan2(deltax, deltay);
        double cosd = cos(estimate_effect_direction);
        estimate_effect_speed = (cosd > 0.1) ? deltay / cosd : deltax / sin(estimate_effect_direction);
        if (estimate_effect_direction < 0)
            estimate_effect_direction = fmod(estimate_effect_direction + M_PI * 10000, M_PI * 2);
        else if (estimate_effect_direction > 2 * M_PI)
            estimate_effect_direction = fmod(estimate_effect_direction, M_PI * 2);
    }
    if(debug)
    {
        estimate_effect_speed = 0;
    }
    //  estimate_effect_direction = 1.57;
    //     estimate_effect_speed = 1;
//    cerr << "current estimate " << estimate_effect_speed << " " << estimate_effect_direction << endl;
//    double choosex, choosey;

    for (int i = -10; i <= 10; ++i)
    {

        rudder = i / 10.0;

        for (int j = 100; j >= 0; --j)
        {
            throttle = j / 100.0;

            double x1 = x, y1 = y, heading1 = heading, speed1 = speed, starttime = 0, rpm1 = rpm, d_time, temp = 0;
            vector<pointc> tempfuture;
            for (int index = 0; index < 4; index++)
            {
                d_time = actionsCopy[index].otime - startCopy.otime;
                if(d_time < 0)
                    continue;
                while (starttime + duration < d_time)
                {
                    starttime += duration;
                    estimate(rpm1, throttle, duration, speed1, rudder, heading1, x1, y1);
                    if (tempfuture.size() < 10)
                    {
                        tempfuture.emplace_back(x1, y1, startCopy.otime + starttime);
                    }
                    // cerr << rudder<< " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << endl;
                }
                if (starttime != d_time)
                {
                    estimate(rpm1, throttle, d_time - starttime, speed1, rudder, heading1, x1, y1);
                    //cerr << rudder<< " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << endl;
                }
                temp += (pow(x1 - actionsCopy[index].x, 2) + pow(y1 - actionsCopy[index].y, 2)) * probability[index];
            }
            if (coefficient > temp)
            {
                r = (int)(rudder * 1000.0) / 1000.0;
                t = (int)(throttle * 1000.0) / 1000.0;
                coefficient = temp;
                future = tempfuture;
            }
            //cerr << temp << " " << rudder << " " << throttle << " " << x1 << " " << y1 << endl;
        }
    }
    //cerr << "action " << action.x << " " << action.y << " target " << choosex << " " << choosey << endl;
    //cerr << "choose coeff " << coefficient << " " << r << " " << t << "\n\n"
      //   << endl;
}

void Controller::sendAction()
{
    double rudder, throttle;
    while (running)
    {
        if (getppid() == 1) // when could this happen??
        {
            cerr << "CONTROLER TERMINATE" << endl;
            exit(1);
        }
        if (plan) //use mutex instead of busy waiting // you mean CV?
        {
            mtx.lock();
            double s = actions[0].speed; // not a great way of indicating an error
            mtx.unlock();
            if (s == -1)
            {
                // No plan? (idk it's an error case)
                update = false;
                m_ControlReceiver->receiveControl(0, 0);
            }
            else if (s == -2)
            {
                // The other error case (no points left to cover?)
                update = false;
                m_ControlReceiver->receiveControl(1, 0.1);
            }
            else
            {
                // actually do MPC
                MPC(rudder, throttle);
                update = true;
//                cerr << "rudder: " << rudder << endl;
                m_ControlReceiver->receiveControl(rudder, throttle);
            }
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Controller::startRunning()
{
    running = true;
    thread thread_for_mpc(thread([=] { sendAction(); }));
    thread_for_mpc.detach();
}

void Controller::terminate()
{
    running = false;
}

void Controller::startSendingControls()
{
    plan = true;
}

void Controller::stopSendingControls()
{
    plan = false;
}


