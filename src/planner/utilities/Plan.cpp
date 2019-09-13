#include "Plan.h"

void Plan::append(const State &s) {
    if (m_States.empty() ||
            (m_States.front().timeUntil(m_States.back()) < Plan::timeHorizon() &&
            m_States.back().timeUntil(s) >= Plan::planTimeDensity())) {
        m_States.emplace_back(s);
    }
}

void Plan::append(const Plan &plan) {
    for (auto s : plan.getRef()) append(s);
}

std::vector<State> Plan::get() const {
    return m_States;
}

std::string Plan::toString() const {
    std::string r;
    for (const auto & m_State : m_States) r += m_State.toString() + "\n";
    return r;
}

const std::vector<State> &Plan::getRef() const {
    return m_States;
}
