#include "DynamicObstacle.h"

DynamicObstacle::DynamicObstacle(State state) : DynamicObstacle(state, c_DefaultWidth, c_DefaultLength) {}

DynamicObstacle::DynamicObstacle(State state, double (* covariance)[4]):
    DynamicObstacle(state, covariance, c_DefaultWidth, c_DefaultLength) {}

DynamicObstacle::DynamicObstacle(State state, double width, double length) : m_State(state) {
    for (auto & row : m_Covariance)
        for (double & i : row)
            i = 0;
    m_Width = width;
    m_Length = length;
}

DynamicObstacle::DynamicObstacle(State state, double (* covariance)[4], double width, double length) :
    m_State(state) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            m_Covariance[i][j] = covariance[i][j];
    m_Width = width;
    m_Length = length;
}

void DynamicObstacle::update(const State& state) {
    m_State = state;
}

double DynamicObstacle::distanceToEdge(double x, double y, double speed, double time) {
    // This is a primitive implementation and should be updated to include an uncertainty buffer
    auto dx = m_State.x - x;
    auto dy = m_State.y - y;
    auto d = sqrt(dx*dx + dy*dy);
    auto t = d / (m_State.speed + speed);
    return t * speed;
}
