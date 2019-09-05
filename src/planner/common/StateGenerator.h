#ifndef SRC_STATEGENERATOR_H
#define SRC_STATEGENERATOR_H

#include <random>
#include <path_planner/State.h>

class StateGenerator {
public:
    StateGenerator(double minX, double maxX,
                    double minY, double maxY,
                    double minSpeed, double maxSpeed,
                    unsigned long seed);

    State generate();

private:
    std::uniform_real_distribution<double> m_XDistribution, m_YDistribution, m_HeadingDistribution, m_SpeedDistribution;
    std::default_random_engine m_RandomEngine;
};


#endif //SRC_STATEGENERATOR_H
