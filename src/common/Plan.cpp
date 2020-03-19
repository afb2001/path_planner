#include <cassert>
#include "Plan.h"

//void Plan::append(const State &s) {
//    if (m_States.empty() ||
//            (m_States.front().timeUntil(m_States.back()) < Plan::timeHorizon() &&
//            m_States.back().timeUntil(s) >= Plan::planTimeDensity())) {
//        m_States.emplace_back(s);
//    }
//}

void Plan::append(const Plan &plan) {
    for (auto s : plan.m_DubinsPaths) append(s);
}

//std::vector<State> Plan::get() const {
//    return m_States;
//}

//std::string Plan::toString() const {
//    std::string r;
//    for (const auto & m_State : m_States) r += m_State.toString() + "\n";
//    return r;
//}

void Plan::append(const DubinsWrapper& dubinsPath) {
    m_DubinsPaths.push_back(dubinsPath);
}

void Plan::sample(State& s) const {
    for (const auto& p : m_DubinsPaths) {
        if (p.containsTime(s.time())) {
            p.sample(s);
            return;
        }
    }
    assert(false && "Requested time outside plan bounds");
}

Plan::Plan(const State& s1, const State& s2, double rho) {
    m_DubinsPaths.emplace_back(s1, s2, rho);
}

bool Plan::empty() const {
    return m_DubinsPaths.empty();
}

std::vector<State> Plan::getHalfSecondSamples() const {
    // should check for duplicates
    std::vector<State> result;
    for (const auto& p : m_DubinsPaths) {
        auto r = p.getSamples(planTimeDensity());
        for (const auto& s : r) result.push_back(s);
    }
    return result;
}

const std::vector<DubinsWrapper>& Plan::get() const {
    return m_DubinsPaths;
}

double Plan::totalTime() const {
    if (empty()) return 0;
    return m_DubinsPaths.back().getEndTime() - m_DubinsPaths.front().getStartTime();
}

bool Plan::containsTime(double time) const {
    for (const auto& p : m_DubinsPaths) if (p.containsTime(time)) return true;
    return false;
}

double Plan::getStartTime() const {
    assert(!m_DubinsPaths.empty());
    return m_DubinsPaths.front().getStartTime();
}

double Plan::getEndTime() const {
    assert(!m_DubinsPaths.empty());
    return m_DubinsPaths.back().getEndTime();
}
