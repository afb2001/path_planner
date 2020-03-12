#include <cassert>
#include "DubinsWrapper.h"

DubinsWrapper::DubinsWrapper(const State& s1, const State& s2, double rho) {
    set(s1, s2, rho);
}

void DubinsWrapper::set(const State& s1, const State& s2, double rho) {
    // until we change State to use yaw internally...
    double q1[3] = {s1.x(), s1.y(), s1.yaw()};
    double q2[3] = {s2.x(), s2.y(), s2.yaw()};
    dubins_shortest_path(&m_DubinsPath, q1, q2, rho);
    m_Speed = s1.speed();
    m_StartTime = s1.time();
    setEndTime();
}

double DubinsWrapper::length() const {
    assert(isInitialized());
    return dubins_path_length(&m_DubinsPath);
}

bool DubinsWrapper::containsTime(double time) const {
    return m_StartTime <= time && m_EndTime >= time;
}

void DubinsWrapper::sample(State& s) const {
    double distance = (s.time() - m_StartTime) * m_Speed;
    // heading comes back as yaw
    dubins_path_sample(&m_DubinsPath, distance, s.pose());
    // set yaw with heading value to correct things
    s.yaw(s.heading()); // TODO! -- change state to internally use yaw?
    s.speed() = m_Speed; // take note of this
}

bool DubinsWrapper::isInitialized() const {
    return m_StartTime > 0;
}

std::vector<State> DubinsWrapper::getSamples(double timeInterval) const {
    std::vector<State> result;
    State intermediate;
    intermediate.speed() = m_Speed;
    for (auto s = m_StartTime; s < m_EndTime; s+= timeInterval) {
        intermediate.time() = s;
        sample(intermediate);
        result.push_back(intermediate);
    }
    return result;
}

double DubinsWrapper::getRho() const {
    return m_DubinsPath.rho;
}

const DubinsPath& DubinsWrapper::unwrap() const {
    return m_DubinsPath;
}

double DubinsWrapper::getStartTime() const {
    return m_StartTime;
}

double DubinsWrapper::getSpeed() const {
    return m_Speed;
}

void DubinsWrapper::fill(const DubinsPath& path, double speed, double startTime) {
    m_DubinsPath = path;
    m_Speed = speed;
    m_StartTime = startTime;
    setEndTime();
}

void DubinsWrapper::setEndTime() {
    m_EndTime = m_StartTime + length() / m_Speed;
}

double DubinsWrapper::getEndTime() const {
    return m_EndTime;
}

void DubinsWrapper::updateEndTime(double endTime) {
    assert(m_EndTime != -1);
    assert(endTime <= m_EndTime);
    m_EndTime = endTime;
}
