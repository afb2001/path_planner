#include "StateGenerator.h"

StateGenerator::StateGenerator(double minX, double maxX, double minY, double maxY, double minSpeed, double maxSpeed,
                                 unsigned long seed) {
    m_XDistribution = std::uniform_real_distribution<>(minX, maxX);
    m_YDistribution = std::uniform_real_distribution<>(minY, maxY);
    m_HeadingDistribution = std::uniform_real_distribution<>(0, 2 * M_PI);
    m_SpeedDistribution = std::uniform_real_distribution<>(minSpeed, maxSpeed);

    m_RandomEngine.seed(seed);
}

State StateGenerator::generate() {
    return State(m_XDistribution(m_RandomEngine),
            m_YDistribution(m_RandomEngine),
            m_HeadingDistribution(m_RandomEngine),
            m_SpeedDistribution(m_RandomEngine),
            0);
}
