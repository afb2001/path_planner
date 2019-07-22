#include "DynamicObstacle.h"

DynamicObstacle::DynamicObstacle(State state) : DynamicObstacle(state, c_DefaultWidth, c_DefaultLength) {}

//DynamicObstacle::DynamicObstacle(State state, double (* covariance)[4]):
//    DynamicObstacle(state, covariance, c_DefaultWidth, c_DefaultLength) {}

DynamicObstacle::DynamicObstacle(State state, double stdDev, double stdDevChangePerSecond) : m_State(state) {
//    for (auto & row : m_Covariance)
//        for (double & i : row)
//            i = 0;
    m_InitialStdDev = stdDev;
    m_StdDevChangePerSecond = stdDevChangePerSecond;
}

//DynamicObstacle::DynamicObstacle(State state, double (* covariance)[4], double width, double length) :
//    m_State(state) {
//    for (int i = 0; i < 4; i++)
//        for (int j = 0; j < 4; j++)
//            m_Covariance[i][j] = covariance[i][j];
//    m_InitialStdDev = width;
//    m_StdDevChangePerSecond = length;
//}

void DynamicObstacle::update(const State& state) {
    m_State = state;
}

double DynamicObstacle::distanceToEdge(double x, double y, double speed, double time) const {
    // This is a primitive implementation and should be updated to include an uncertainty buffer
    // also this is wrong (doesn't use given time)
    auto dx = m_State.x - x;
    auto dy = m_State.y - y;
    auto d = sqrt(dx*dx + dy*dy);
    auto t = d / (m_State.speed + speed);
    return t * speed;
}

double DynamicObstacle::probabilityOfCollisionAt(double x, double y, double time) const {
    auto deltaT = time - m_State.time;
    // just project initial x and y into the future with the approximate heading and speed
    auto xMean = m_State.x + deltaT * m_State.speed * sin(m_State.heading);
    auto yMean = m_State.y + deltaT * m_State.speed * cos(m_State.heading);
    // assume independence and equal stdDev for x and y
    auto stdDev = m_InitialStdDev + deltaT * m_StdDevChangePerSecond;
    // use the Gaussian probability density function to calculate probability in each dimension
    auto xProb = normal_pdf(x, xMean, stdDev);
    auto yProb = normal_pdf(y, yMean, stdDev);
//    auto xProb = exp(-((x - xMean)*(x - xMean)) / (2 * stdDev)) / sqrt(2 * M_PI * stdDev);
//    auto yProb = exp(-((y - yMean)*(y - yMean)) / (2 * stdDev)) / sqrt(2 * M_PI * stdDev);
    // joint probability of independent events is the product of their probabilities
    return xProb * yProb;
}

void DynamicObstacle::update(const State& state, double stdDev, double stdDevChangePerSecond) {
    m_State = state;
    m_InitialStdDev = stdDev;
    m_StdDevChangePerSecond = stdDevChangePerSecond;
}

DynamicObstacle::DynamicObstacle() {
    m_InitialStdDev = 0;
    m_StdDevChangePerSecond = 0;
}
