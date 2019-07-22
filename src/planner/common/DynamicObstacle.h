#ifndef SRC_DYNAMICOBSTACLE_H
#define SRC_DYNAMICOBSTACLE_H


#include <path_planner/State.h>

class DynamicObstacle {
public:
    DynamicObstacle();
    explicit DynamicObstacle(State state);
    DynamicObstacle(State state, double stdDev, double stdDevChangePerSecond);
//    DynamicObstacle(State state, double covariance[4][4]);
//    DynamicObstacle(State state, double covariance[4][4], double width, double length);

    void update(const State& state);
    void update(const State& state, double stdDev, double stdDevChangePerSecond);

    double distanceToEdge(double x, double y, double speed, double time) const;

    double probabilityOfCollisionAt(double x, double y, double time) const;

private:
    State m_State;
    double m_InitialStdDev, m_StdDevChangePerSecond;
//    double m_Covariance[4][4]; // x, y, heading, speed covariance matrix

    static constexpr double c_DefaultWidth = 3, c_DefaultLength = 3;

    template <typename T>
    static T normal_pdf(T x, T mean, T stdDev)
    {
        static const T inv_sqrt_2pi = 0.3989422804014327;
        T a = (x - mean) / stdDev;

        return inv_sqrt_2pi / stdDev * std::exp(-T(0.5) * a * a);
    }
};


#endif //SRC_DYNAMICOBSTACLE_H
