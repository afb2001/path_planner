#include "RibbonManager.h"

#include <cfloat>
#include <sstream>
#include <vector>

void RibbonManager::add(double x1, double y1, double x2, double y2) {
    if (m_Ribbons.size() > c_RibbonCountDangerThreshold)
        std::cerr << "Warning: adding more ribbons than can be used for TSP heuristics" << std::endl;
    Ribbon r(x1, y1, x2, y2);
    add(r, m_Ribbons.end(), false);
}

void RibbonManager::cover(double x, double y, bool strict) {
    auto i = m_Ribbons.begin();
    while (i != m_Ribbons.end()) {
        auto r = i->split(x, y, strict);
        add(r, i, strict);
        if (i->covered(strict)) i = m_Ribbons.erase(i);
        else ++i;
    }
}

bool RibbonManager::done() const {
    return m_Ribbons.empty();
}

double RibbonManager::approximateDistanceUntilDone(double x, double y, double yaw) const {
   if (done()) return 0;
    // if we're above the danger threshold just give max distance
//    if (m_Ribbons.size() > c_RibbonCountDangerThreshold) return maxDistance(x, y);
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
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        const Ribbon r = *it;
        it = ribbonsLeft.erase(it);
        min = fmin(min, tspPointRobotNoSplitAllRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() -
            2 * Ribbon::RibbonWidth + distance(point, r.start()), 0), r.end()));
        min = fmin(min, tspPointRobotNoSplitAllRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() -
            2 * Ribbon::RibbonWidth + distance(point, r.end()), 0), r.start()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}

double RibbonManager::tspPointRobotNoSplitKRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar,
                                                   std::pair<double, double> point) const {
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
        const Ribbon r = *it;
        it = ribbonsLeft.erase(it);
        min = fmin(min, tspPointRobotNoSplitKRibbons(ribbonsLeft, fmax(distanceSoFar + r.length()-
            2 * Ribbon::RibbonWidth + distance(point, r.start()), 0), r.end()));
        min = fmin(min, tspPointRobotNoSplitKRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() -
            2 * Ribbon::RibbonWidth + distance(point, r.end()), 0), r.start()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}


double RibbonManager::tspDubinsNoSplitAllRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, double x,
                                                 double y, double yaw) const{
    // Depth-first TSP solution
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        const auto r = *it;
        it = ribbonsLeft.erase(it);
        auto start = r.startAsState();
        auto end = r.endAsState();
        min  = fmin(min, tspDubinsNoSplitAllRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() - 2 * Ribbon::RibbonWidth +
            dubinsDistance(x, y, yaw, start), 0), end.x(), end.y(), end.yaw()));
        min  = fmin(min, tspDubinsNoSplitAllRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() - 2 * Ribbon::RibbonWidth +
            dubinsDistance(x, y, yaw, end), 0), start.x(), start.y(), start.yaw()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}

double RibbonManager::tspDubinsNoSplitKRibbons(std::list<Ribbon> ribbonsLeft, double distanceSoFar, double x, double y,
                                               double yaw) const {
    if (ribbonsLeft.empty()) return distanceSoFar;
    auto min = DBL_MAX;
    auto comp = [&] (const Ribbon& r1, const Ribbon& r2) {
        double min1 = fmin(dubinsDistance(x, y, yaw, r1.startAsState()), dubinsDistance(x, y, yaw, r1.endAsState()));
        double min2 = fmin(dubinsDistance(x, y, yaw, r1.startAsState()), dubinsDistance(x, y, yaw, r1.endAsState()));
        return min1 > min2;
    };
    int i = 0;
    ribbonsLeft.sort(comp);
    for (auto it = ribbonsLeft.begin(); it != ribbonsLeft.end(); it++) {
        if (i >= m_K) break;
        const auto r = *it;
        it = ribbonsLeft.erase(it);
        auto start = r.startAsState();
        auto end = r.endAsState();
        min  = fmin(min, tspDubinsNoSplitKRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() - 2 * Ribbon::RibbonWidth +
                                                                 dubinsDistance(x, y, yaw, start), 0), end.x(), end.y(), end.yaw()));
        min  = fmin(min, tspDubinsNoSplitKRibbons(ribbonsLeft, fmax(distanceSoFar + r.length() - 2 * Ribbon::RibbonWidth +
                                                                 dubinsDistance(x, y, yaw, end), 0), start.x(), start.y(), start.yaw()));
        it = ribbonsLeft.insert(it, r);
    }
    return min;
}

double RibbonManager::minDistanceFrom(double x, double y) const {
    if (m_Ribbons.empty()) return 0;
    auto min = DBL_MAX;
    for (const auto& r : m_Ribbons) {
        if (r.contains(x, y, r.getProjection(x, y), false)) return 0;
        auto dStart = distance(r.start(), x, y);
        auto dEnd = distance(r.end(), x, y);
        min = fmin(fmin(min, dEnd), dStart);
    }
    return min;
}

void RibbonManager::add(const Ribbon& r, std::list<Ribbon>::iterator i, bool strict) {
    if (r.covered(strict)) return;
    // TODO! -- determine whether to split any of the prior ribbons based on this new one
    m_Ribbons.insert(i, r);
}

State RibbonManager::getNearestEndpointAsState(const State& state) const {
    if (done()) throw std::logic_error("Attempting to get nearest endpoint when there are no ribbons");
    auto min = DBL_MAX;
    State ret;
    for (const auto& r : m_Ribbons) {
        auto s = r.startAsState();
        s.move(Ribbon::minLength() / Ribbon::strictModifier() + 1e-5);
        auto d = state.distanceTo(s);
        if (d < min) {
            if (d < Ribbon::minLength()){ // && r.contains(state.x(), state.y(), r.getProjection(state.x(), state.y))) {
                // we actually want the state at the other end of the ribbon
                ret = r.endAsState();
                ret.heading() = s.heading();
                ret.move(-Ribbon::minLength() / Ribbon::strictModifier() + 1e-5); // pull back up the ribbon a little because technically the ribbon can end here
            } else {
                ret = s;
            }
            min = d;
        }
        s = r.endAsState();
        s.move(Ribbon::minLength() / Ribbon::strictModifier() + 1e-5);
        d = state.distanceTo(s);
        if (d < min) {
            if (d < Ribbon::minLength()){ // && r.contains(state.x(), state.y(), r.getProjection(state.x(), state.y))) {
                // we actually want the state at the other end of the ribbon
                ret = r.startAsState();
                ret.heading() = s.heading();
                ret.move(-Ribbon::minLength() / Ribbon::strictModifier() + 1e-5); // pull back up the ribbon a little because technically the ribbon can end here
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
        auto d = r.distance(state.x(), state.y());
        if (d < min) {
            min = d;
            ribbon = r;
        }
    }
    state = ribbon.getProjectionAsState(state.x(), state.y());
}

double RibbonManager::maxDistance(double x, double y) const {
    // max represents the distance to the farthest endpoint.
    // min represents the distance to the nearest endpoint plus the sum of the lengths of all ribbons.
    // Whichever is larger is returned.
    // Both are technically inadmissible due to the "done" action but that's not implemented yet anywhere
    double sumLength = 0, min = DBL_MAX, max = 0;
    for (const auto& r : m_Ribbons) {
        sumLength += r.length() - 2 * Ribbon::RibbonWidth; // can technically shortcut the ribbon on both ends
        auto dStart = distance(r.start(), x, y);
        auto dEnd = distance(r.end(), x, y);
        min = fmin(fmin(min, dEnd), dStart);
        max = fmax(fmax(max, dEnd), dStart);
    }
    return fmax(sumLength + min, max);
}

const std::list<Ribbon>& RibbonManager::get() const {
    return m_Ribbons;
}

std::vector<State> RibbonManager::findStatesOnRibbonsOnCircle(const State& center, double radius) const {
    std::vector<State> states;
    for (const auto& r : m_Ribbons) {
        // circle line intersection from mathworld.wolfram.com
        auto dx = r.end().first - r.start().first;
        auto dy = r.end().second - r.start().second;
        auto dr = sqrt(dx*dx + dy*dy);
        auto d = r.start().first*r.end().second - r.end().first*r.start().second;
        auto i1 = dr*dr;
        auto discriminant = radius*radius*i1 - d*d;
        if (discriminant < 0) continue; // no intersection
        auto i2 = sqrt(discriminant);
        double sgn = dy < 0? -1 : 1;
        auto i3 = sgn * dx * i2;
        auto i4 = d * dy;
        auto x1 = (i4 + i3) / i1;
        auto x2 = (i4 - i3) / i1;
        auto i5 = -d * dx;
        auto i6 = fabs(dy) * i2;
        auto y1 = (i5 + i6) / i1;
        auto y2 = (i5 - i6) / i1;
        const auto start = r.startAsState();
        const auto end = r.endAsState();
        // took out checks to determine the points are actually in the ribbons because it might make sense to try to
        // drive to points past the ribbons anyway, and if these are only used once it won't matter much
        // EDIT -- put them back in (they're the if (r.contains(...)) checks)
        if (r.contains(x1, y1, r.getProjection(x1, y1), false)) {
            // give each intersecting point both headings
            states.emplace_back(x1, y1, start.heading(), start.speed(), 0);
            states.emplace_back(x1, y1, end.heading(), end.speed(), 0);
        }
        // if it's a tangent line then they will be the same point
        if (x1 != x2 && y1 != y2) {
            if (r.contains(x1, y1, r.getProjection(x2, y2), false)) {
                states.emplace_back(x2, y2, start.heading(), start.speed(), 0);
                states.emplace_back(x2, y2, end.heading(), end.speed(), 0);
            }
        }
    }
    return states;
}

std::vector<State> RibbonManager::findNearStatesOnRibbons(const State& start, double radius) const {
    std::vector<State> states;
    auto h = start.yaw() + M_PI_2;
    // get points one radius away from start in the directions perpendicular to its heading
    auto x1 = start.x() + cos(h) * radius;
    auto x2 = start.x() - cos(h) * radius;
    auto y1 = start.y() + sin(h) * radius;
    auto y2 = start.y() - sin(h) * radius;

    for (const Ribbon& r : m_Ribbons) {

        // check if ribbon is anywhere near current state (within 2*r)
        auto startProj = r.getProjection(start.x(), start.y());
        { // scope to clearly mark usage of poorly named "d"
            double d;
            if (r.containsProjection(startProj)) {
                d = start.distanceTo(startProj.first, startProj.second);
            } else {
                d = fmin(start.distanceTo(r.start().first, r.start().second),
                         start.distanceTo(r.end().first, r.end().second));
            }
            if (d > 2 * radius) continue;
        }

        // project points from before onto ribbon
        auto proj1 = r.getProjection(x1, y1);
        auto proj2 = r.getProjection(x2, y2);
        auto proj = proj2;
        auto x = x2; auto y = y2;
        if (r.containsProjection(proj1)) {
            proj = proj1;
            x = x1; y = y1;
        }

        // go another half radius out
        auto s1 = r.startAsState();
        auto s2 = r.endAsState();
        State s;
        if (s1.distanceTo(start) < s2.distanceTo(start)) {
            s = s1;
        } else {
            s = s2;
        }
        auto h2 = s.yaw() - M_PI_2; // might not be right in all cases but seems to work in practice
        auto dx1 = cos(h2) * radius / 2;
        auto dy1 = sin(h2) * radius / 2;
//        auto x3 = proj.first + proj.first < x ? dx1 : -dx1;
        auto x3 = proj.first + dx1;
//        auto y3 = proj.second = proj.second < y ? dy1 : -dy1;
        auto y3 = proj.second + dy1;
//        auto x4 = proj2.first = proj2.first < x2 ? dx1 : -dx1;
//        auto y4 = proj2.second = proj2.second < y2 ? dy1 : -dy1;

        // extend that ahead until it reaches the circle around (x1, y1)
        // sqrt(a^2 - r^2)
        auto a = dx1 * dx1 + dy1 * dy1;
        auto rSquared = radius * radius;
        auto b = sqrt(rSquared - a);
        auto h3 = s.yaw();
        auto x5 = x3 + b * cos(h3);
        auto y5 = y3 + b * sin(h3);
//        auto x6 = x4 + b * cos(h3);
//        auto y6 = y4 + b * sin(h3);

        // go one radius along the line from (x1, y1) to that point
        auto x7 = x5 - x;
        auto y7 = y5 - y;
        auto h4 = atan(y7 / x7);
        auto x8 = x5 + radius * cos(h4);
        auto y8 = y5 + radius * sin(h4);

        // project that back onto the ribbon
        auto projFinal = r.getProjection(x8, y8);

        // done. that's the state. If it's close by add it to the list
        auto d = distance(projFinal, start.x(), start.y());
        if (d > 1e-5 && d < 2 * radius){
            states.emplace_back(projFinal.first, projFinal.second, s.heading(), 0, 0);
//            std::cerr << "Found Brown path to state " << states.back().toString() << " from " << start.toString() << std::endl;
//            states.back().push(0.1); // push the state along the ribbon a tiny bit to fix rounding errors
        }
    }
    return states;
}

void RibbonManager::changeHeuristicIfTooManyRibbons() {
    if (m_Ribbons.size() > c_RibbonCountDangerThreshold) {
        m_Heuristic = MaxDistance;
    }
}

void RibbonManager::setHeuristic(Heuristic heuristic) {
    m_Heuristic = heuristic;
}

void RibbonManager::coverBetween(double x1, double y1, double x2, double y2, bool strict) {
    double theta = atan((y2 - y1) / (x2 - x1));
    double d = distance(x1, y1, x2, y2);
    do {
        auto d1 = distance(x1, y1, x2, y2);
        if (d1 > d) break; // ensure distance to go is decreasing
        else d = d1;
        cover(x1, y1, strict);
        x1 += Ribbon::minLength() * cos(theta) / 2; // so we don't overshoot
        y1 += Ribbon::minLength() * sin(theta) / 2;
    } while (d > Ribbon::minLength());
    cover(x2, y2, strict);
}

double RibbonManager::coverageCompletedTime() const {
    return m_CoverageCompletedTime;
}

void RibbonManager::setCoverageCompletedTime(double coverageCompletedTime) {
    if (m_CoverageCompletedTime == -1)
        m_CoverageCompletedTime = coverageCompletedTime;
}

double RibbonManager::getTotalUncoveredLength() const {
    auto sum = 0;
    for (const auto& r : m_Ribbons) sum += r.length();
    return sum;
}

