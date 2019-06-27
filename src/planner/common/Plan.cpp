#include "Plan.h"

void Plan::append(const State &s) {
    if (m_States.empty() ||
            (m_States.front().timeUntil(m_States.back()) < TIME_HORIZON &&
            m_States.back().timeUntil(s) >= PLAN_TIME_DENSITY)) {
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
    std::string r = "";
    for (int i = 0; i < m_States.size(); i++) r += m_States[i].toString() + "\n";
    return r;
}

const std::vector<State> &Plan::getRef() const {
    return m_States;
}
