//
// Created by alex on 4/22/19.
//

#ifndef SRC_EXECUTIVE_H
#define SRC_EXECUTIVE_H

#include <string.h>
#include "communication.h"
#include "path.h"
#include "controller.h"


class Executive
{
public:

    explicit Executive(ControlReceiver *controlReceiver);

    ~Executive();

    void updateCovered(double x, double y, double speed, double heading, double t);

    void addToCover(int x, int y);

//    void run(int argc, char* argv[]);

    void terminate();

    void startController();

    void startPlanner(std::string mapFile);

private:

    bool running = true;
    bool request_start = false;
    Path path;

    bool debug = true;
    bool pause_all = true;

    Communication communication_With_Planner;

    Controller* m_Controller;

    ControlReceiver* m_ControlReceiver;

    void checkTerminate();

    static double getCurrentTime();

    void requestPath();

    /**
     * Signal to the node that we're done and pause everything until we make a new planner instance.
     */
    void finished();

// TODO! -- Add a way to update obstacles

//    void requestWorldInformation();

    void sendAction();

    void print_map(std::string file);

//    void read_goal(std::string goal);
};

#endif //SRC_EXECUTIVE_H
