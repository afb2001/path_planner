#include <path_planner_common/State.h>
#include <path_planner_common/DubinsWrapper.h>

#include <sstream>

extern "C" {
#include <dubins.h>
}

DubinsWrapper::DubinsWrapper(const State& s1, const State& s2, double rho) {
    set(s1, s2, rho);
}

void DubinsWrapper::set(const State& s1, const State& s2, double rho) {
    // until we change State to use yaw internally...
    double q1[3] = {s1.x(), s1.y(), s1.yaw()};
    double q2[3] = {s2.x(), s2.y(), s2.yaw()};
    dubins_shortest_path(&m_DubinsPath, q1, q2, rho);
    m_Speed = s1.speed();
    m_UpdatedStartTime = m_StartTime = s1.time();
    setEndTime();
}

double DubinsWrapper::length() const {
    if (!isInitialized()) throw std::runtime_error("Cannot access unset Dubins wrapper");
    return dubins_path_length(&m_DubinsPath);
}

bool DubinsWrapper::containsTime(double time) const {
    if (!isInitialized()) throw std::runtime_error("Checking time constraints on uninitialized Dubins wrapper");
    return m_UpdatedStartTime <= time && m_EndTime >= time;
}

void DubinsWrapper::sample(State& s) const {
    if (!containsTime(s.time())) {
        std::stringstream stream;
        stream << "Invalid time " << std::to_string(s.time()) << " in sample for Dubins path which spans from "
            << std::to_string(getStartTime()) << " to " << std::to_string(getEndTime());
        throw std::runtime_error(stream.str());
    }
    double distance = (s.time() - m_StartTime) * m_Speed;
    // heading comes back as yaw
    int err = dubins_path_sample(&m_DubinsPath, distance, s.pose());
    if (err == EDUBPARAM) {
        // rounding error sometimes makes us overshoot the length, which causes an error
        err = dubins_path_sample(&m_DubinsPath, distance - 1e-5, s.pose());
    }
    if (err != EDUBOK) {
        std::cerr << "Encountered error in dubins library" << std::endl;
    }
    // set yaw with heading value to correct things
    s.setYaw(s.heading()); // TODO! -- change state to internally use yaw?
    s.speed() = m_Speed; // take note of this
}

bool DubinsWrapper::isInitialized() const {
    return m_StartTime >= 0;
}

std::vector<State> DubinsWrapper::getSamples(double timeInterval, double offset) const {
    // TODO! -- offset
    // deprecated
    std::vector<State> result;
    State intermediate;
    intermediate.speed() = m_Speed;
    for (auto s = m_UpdatedStartTime; s < m_EndTime; s+= timeInterval) {
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
    return m_UpdatedStartTime;
}

double DubinsWrapper::getSpeed() const {
    return m_Speed;
}

void DubinsWrapper::fill(const DubinsPath& path, double speed, double startTime) {
    m_DubinsPath = path;
    m_Speed = speed;
    m_UpdatedStartTime = m_StartTime = startTime;
    setEndTime();
}

void DubinsWrapper::setEndTime() {
    m_EndTime = m_StartTime + length() / m_Speed;
}

double DubinsWrapper::getEndTime() const {
    return m_EndTime;
}

void DubinsWrapper::updateEndTime(double endTime) {
    if (m_EndTime == -1) throw std::runtime_error("Cannot access unset Dubins wrapper");
    if(endTime > m_EndTime) throw std::runtime_error("Invalid end time for Dubins wrapper");
    m_EndTime = endTime;
}

void DubinsWrapper::updateStartTime(double startTime) {
    if (!isInitialized()) throw std::runtime_error("Cannot access unset Dubins wrapper");
    if (startTime < m_StartTime) throw std::runtime_error("Invalid start time for Dubins wrapper");
    m_UpdatedStartTime = startTime;
    // literally move it (thankfully Dubins lib has function for this)
    auto d = (m_UpdatedStartTime - m_StartTime) * m_Speed;
    m_StartTime = startTime;
    auto copy = m_DubinsPath;
    dubins_extract_subpath(&copy, d, &m_DubinsPath);
}

double DubinsWrapper::getNetTime() const {
    return getEndTime() - getStartTime();
}

void DubinsWrapper::setSpeed(double speed) {
    m_Speed = speed;
    setEndTime();
}
