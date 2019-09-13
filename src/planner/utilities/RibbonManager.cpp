#include <cfloat>
#include <algorithm>
#include <sstream>
#include <vector>
#include "RibbonManager.h"

void RibbonManager::add(double x1, double y1, double x2, double y2) {
    if (m_Ribbons.size() > c_RibbonCountDangerThreshold)
        std::cerr << "Warning: adding more ribbons than can be used for TSP heuristics" << std::endl;
    Ribbon r(x1, y1, x2, y2);
    add(r, m_Ribbons.end());
}

void RibbonManager::cover(double x, double y) {
    auto i = m_Ribbons.begin();
    while (i != m_Ribbons.end()) {
        auto r = i->split(x, y);
        add(r, i);
        if (i->covered()) i = m_Ribbons.erase(i);
        else ++i;
    }
}

bool RibbonManager::done() const {
    return m_Ribbons.empty();
}

double RibbonManager::approximateDistanceUntilDone(double x, double y, double yaw) {
    if (done()) return 0;
    // if we're above the danger threshold just give max distance
    if (m_Ribbons.size() > c_RibbonCountDangerThreshold) return maxDistance(x, y);
    switch (m_Heuristic) {
        // Modified max distance heuristic
        case MaxDistance: {
            return maxDistance(x, y);
        }
        case TspPointRobotNoSplitAllRibbons: {
            return tspPointRobotNoSplitAllRibbons(m_Ribbons, 0, std::make_pair(x, y));
        }
        case TspDubinsNoSplitAllRibbons: {
            return tspDubinsNoSplitAllRibbons(m_Ribbons, 0, x, y, yaw);
        }
        case TspPointRobotNoSplitKRibbons: {
            return tspPointRobotNoSplitKRibbons(m_Ribbons, 0, std::make_pair(x, y));
        }
        case TspDubinsNoSplitKRibbons: {
            return tspDubinsNoSplitKRibbons(m_Ribbons, 0, x, y, yaw);
        }
        default: return 0;
    }
}

double RibbonManager::tspPointRobotNoSplitAllRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, std::pair<double, double> point) {
    // Depth-first TSP solution
//    std::cerr << "Computing TSP solution from " << point.first << ", " << point.second << std::endl;
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        const Ribbon r = *it;
        it = ribbonsLeft.erase(it);
        min = fmin(min, tspPointRobotNoSplitAllRibbons(ribbonsLeft, distanceSoFar + r.length() +
                   distance(point, r.start()), r.end()));
        min = fmin(min, tspPointRobotNoSplitAllRibbons(ribbonsLeft, distanceSoFar + r.length() +
                   distance(point, r.end()), r.start()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}

double RibbonManager::tspPointRobotNoSplitKRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar,
                                                   std::pair<double, double> point) {
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    auto comp = [&] (const Ribbon& r1, const Ribbon& r2) {
        double min1 = fmin(distance(point, r1.start()), distance(point, r1.end()));
        double min2 = fmin(distance(point, r2.start()), distance(point, r2.end()));
        return min1 > min2; // should be lt except that make_heap makes a max heap
    };
    // Just sort the list because it's easier than trying to make it a heap.
    // I'm not even sure it would be advantageous to use a heap because we need the constant time insert/delete,
    // of which the heap-supporting containers are incapable, so we'd need to make a copy
    ribbonsLeft.sort(comp);
    int i = 0;
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        if (i++ >= m_K) break;
        // TODO! -- this one and the other K-based one don't work because pop_back()
        const Ribbon r = *it;
        it = ribbonsLeft.erase(it);
        min = fmin(min, tspPointRobotNoSplitKRibbons(ribbonsLeft, distanceSoFar + r.length() +
                                                                    distance(point, r.start()), r.end()));
        min = fmin(min, tspPointRobotNoSplitKRibbons(ribbonsLeft, distanceSoFar + r.length() +
                                                                    distance(point, r.end()), r.start()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}


double RibbonManager::tspDubinsNoSplitAllRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, double x,
                                                 double y, double yaw) {
    // Depth-first TSP solution
//    std::cerr << "Computing TSP solution from " << x << ", " << y << std::endl;
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        const auto r = *it;
        it = ribbonsLeft.erase(it);
        auto start = r.startAsState();
        auto end = r.endAsState();
        min  = fmin(min, tspDubinsNoSplitAllRibbons(ribbonsLeft, distanceSoFar + r.length() +
            dubinsDistance(x, y, yaw, start), end.x, end.y, end.yaw()));
        min  = fmin(min, tspDubinsNoSplitAllRibbons(ribbonsLeft, distanceSoFar + r.length() +
            dubinsDistance(x, y, yaw, end), start.x, start.y, start.yaw()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}

double RibbonManager::tspDubinsNoSplitKRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, double x, double y,
                                               double yaw) {
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    auto comp = [&] (const Ribbon& r1, const Ribbon& r2) {
        double min1 = fmin(dubinsDistance(x, y, yaw, r1.startAsState()), dubinsDistance(x, y, yaw, r1.endAsState()));
        double min2 = fmin(dubinsDistance(x, y, yaw, r1.startAsState()), dubinsDistance(x, y, yaw, r1.endAsState()));
        return min1 > min2;
    };
    int i = 0;
    ribbonsLeft.sort(comp);
//    std::sort(ribbonsLeft.begin(), ribbonsLeft.end(), comp);
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        if (i >= m_K) break;
        const auto r = *it;
        it = ribbonsLeft.erase(it);
        auto start = r.startAsState();
        auto end = r.endAsState();
        min  = fmin(min, tspDubinsNoSplitKRibbons(ribbonsLeft, distanceSoFar + r.length() +
                                                                 dubinsDistance(x, y, yaw, start), end.x, end.y, end.yaw()));
        min  = fmin(min, tspDubinsNoSplitKRibbons(ribbonsLeft, distanceSoFar + r.length() +
                                                                 dubinsDistance(x, y, yaw, end), start.x, start.y, start.yaw()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}

double RibbonManager::minDistanceFrom(double x, double y) {
    if (m_Ribbons.empty()) return 0;
    auto min = DBL_MAX;
    for (const auto& r : m_Ribbons) {
        if (r.contains(x, y, r.getProjection(x, y))) return 0;
        auto dStart = distance(r.start(), x, y);
        auto dEnd = distance(r.end(), x, y);
        min = fmin(fmin(min, dEnd), dStart);
    }
    return min;
}

void RibbonManager::add(const Ribbon& r, std::list<Ribbon>::iterator i) {
    if (r.covered()) return;
    // TODO! -- issue warning about large numbers of ribbons
    // TODO! -- determine whether to split any of the prior ribbons based on this new one
    m_Ribbons.insert(i, r);
}

State RibbonManager::getNearestEndpointAsState(const State& state) const {
    if (done()) throw std::logic_error("Attempting to get nearest endpoint when there are no ribbons");
    auto min = DBL_MAX;
    State ret(0);
    for (const auto& r : m_Ribbons) {
        auto s = r.startAsState();
        auto d = state.distanceTo(s);
        if (d < min) {
            if (d < Ribbon::minLength() && r.contains(state.x, state.y, r.getProjection(state.x, state.y))) {
                ret = r.endAsState();
                ret.heading = s.heading;
            } else {
                ret = s;
            }
            min = d;
        }
        s = r.endAsState();
        d = state.distanceTo(s);
        if (d < min) {
            if (d < Ribbon::minLength() && r.contains(state.x, state.y, r.getProjection(state.x, state.y))) {
                ret = r.startAsState();
                ret.heading = s.heading;
            } else {
                ret = s;
            }
            min = d;
        }
    }
    return ret;
}

RibbonManager::RibbonManager(RibbonManager::Heuristic heuristic) : m_Heuristic(heuristic) {}

RibbonManager::RibbonManager() : RibbonManager(MaxDistance) {}

//}

std::string RibbonManager::dumpRibbons() const {
    std::stringstream stream;
    stream << "Ribbons: \n";
    if (m_Ribbons.empty()) stream << "None\n";
    else for (const auto& r : m_Ribbons) stream << r.toString() << "\n";
    return stream.str();
}

RibbonManager::RibbonManager(RibbonManager::Heuristic heuristic, double turningRadius) : RibbonManager(heuristic) {
    m_TurningRadius = turningRadius;
}

RibbonManager::RibbonManager(RibbonManager::Heuristic heuristic, double turningRadius, int k)
    : RibbonManager(heuristic, turningRadius) {
    m_K = k;
}

void RibbonManager::projectOntoNearestRibbon(State& state) const {
    if (m_Ribbons.empty()) return;
    auto min = DBL_MAX;
    auto ribbon = Ribbon::empty();
    for (const auto& r : m_Ribbons) {
        auto d = r.distance(state.x, state.y);
        if (d < min) {
            min = d;
            ribbon = r;
        }
    }
    state = ribbon.getProjectionAsState(state.x, state.y);
}

double RibbonManager::maxDistance(double x, double y) const {
    // max represents the distance to the farthest endpoint.
    // min represents the distance to the nearest endpoint plus the sum of the lengths of all ribbons.
    // Whichever is larger is returned.
    // Both are technically inadmissible due to the "done" action but that's not implemented yet anywhere
    double sumLength = 0, min = DBL_MAX, max = 0;
    for (const auto& r : m_Ribbons) {
        sumLength += r.length();
        auto dStart = distance(r.start(), x, y);
        auto dEnd = distance(r.end(), x, y);
        min = fmin(fmin(min, dEnd), dStart);
        max = fmax(fmax(max, dEnd), dStart);
    }
    return fmax(sumLength + min, max);
}
