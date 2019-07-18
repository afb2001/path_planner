//
// Created by abrown on 7/18/19.
//

#ifndef SRC_DYNAMICOBSTACLE_H
#define SRC_DYNAMICOBSTACLE_H


#include <path_planner/State.h>

class DynamicObstacle {
    explicit DynamicObstacle(State state);
    DynamicObstacle(State state, double width, double length);
    DynamicObstacle(State state, double covariance[4][4]);
    DynamicObstacle(State state, double covariance[4][4], double width, double length);

    void update(const State& state);

    double distanceToEdge(double x, double y, double speed, double time);

private:
    State m_State;
    double m_Width, m_Length;
    double m_Covariance[4][4]; // x, y, heading, speed covariance matrix

    static constexpr double c_DefaultWidth = 3, c_DefaultLength = 3;
};


#endif //SRC_DYNAMICOBSTACLE_H
