#include "StateGenerator.h"

#include <utility>

StateGenerator::StateGenerator(double minX, double maxX, double minY, double maxY, double minSpeed, double maxSpeed,
                                 unsigned long seed) {
    m_XDistribution = std::uniform_real_distribution<>(minX, maxX);
    m_YDistribution = std::uniform_real_distribution<>(minY, maxY);
    m_HeadingDistribution = std::uniform_real_distribution<>(0, 2 * M_PI);
    m_SpeedDistribution = std::uniform_real_distribution<>(minSpeed, maxSpeed);

    m_RandomEngine.seed(seed);
}

State StateGenerator::generate() {

    State s = State(m_XDistribution(m_RandomEngine),
                           m_YDistribution(m_RandomEngine),
                           m_HeadingDistribution(m_RandomEngine),
                           m_SpeedDistribution(m_RandomEngine),
                           0);
    if (m_SampleOnRibbons) {
        if (m_HeadingDistribution(m_RandomEngine) < M_PI / 50) { // one in 100 chance
            m_RibbonManager.projectOntoNearestRibbon(s);
            if (m_HeadingDistribution(m_RandomEngine) < M_PI) { // one in two chance
                s.heading += M_PI; // flip the heading (point to the start of the ribbon instead of the end)
            }
        }
    }
    return s;
}

StateGenerator::StateGenerator(double minX, double maxX, double minY, double maxY, double minSpeed, double maxSpeed,
                               unsigned long seed, RibbonManager ribbonManager) 
                               : StateGenerator(minX, maxX, minY, maxY, minSpeed, maxSpeed, seed){
    m_RibbonManager = std::move(ribbonManager);
    m_SampleOnRibbons = true;
}
