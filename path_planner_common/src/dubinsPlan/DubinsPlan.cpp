#include <path_planner_common/DubinsPlan.h>

void DubinsPlan::append(const DubinsPlan &plan) {
    for (auto s : plan.m_DubinsPaths) append(s);
}

void DubinsPlan::append(const DubinsWrapper& dubinsPath) {
    m_DubinsPaths.push_back(dubinsPath);
}

void DubinsPlan::sample(State& s) const {
    for (const auto& p : m_DubinsPaths) {
        if (p.containsTime(s.time())) {
            p.sample(s);
            return;
        }
    }
    throw std::runtime_error("Requested time outside plan bounds");
}

DubinsPlan::DubinsPlan(const State& s1, const State& s2, double rho) {
    m_DubinsPaths.emplace_back(s1, s2, rho);
}

bool DubinsPlan::empty() const {
    return m_DubinsPaths.empty();
}

std::vector<State> DubinsPlan::getHalfSecondSamples() const {
    // should check for duplicates
    std::vector<State> result;
    for (const auto& p : m_DubinsPaths) {
        auto r = p.getSamples(planTimeDensity());
        for (const auto& s : r) result.push_back(s);
    }
    return result;
}

const std::vector<DubinsWrapper>& DubinsPlan::get() const {
    return m_DubinsPaths;
}

double DubinsPlan::totalTime() const {
    if (empty()) return 0;
    return m_DubinsPaths.back().getEndTime() - m_DubinsPaths.front().getStartTime();
}

bool DubinsPlan::containsTime(double time) const {
    for (const auto& p : m_DubinsPaths) if (p.containsTime(time)) return true;
    return false;
}

double DubinsPlan::getStartTime() const {
    if (m_DubinsPaths.empty()) throw std::runtime_error("Cannot access empty plan");
    return m_DubinsPaths.front().getStartTime();
}

double DubinsPlan::getEndTime() const {
    if (m_DubinsPaths.empty()) throw std::runtime_error("Cannot access empty plan");
    return m_DubinsPaths.back().getEndTime();
}

void DubinsPlan::changeIntoSuffix(double time) {
    if (m_DubinsPaths.empty()) throw std::runtime_error("Cannot access empty plan");
    for (auto& p : m_DubinsPaths) {
        if (p.containsTime(time)) {

        }
    }
}
