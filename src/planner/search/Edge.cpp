#include <utility>
#include <algorithm>
#include <memory>
#include "Edge.h"
#include "../utilities/Path.h"
#include <cfloat>

extern "C" {
#include <dubins.h>
}

Edge::Edge(std::shared_ptr<Vertex> start, bool useRibbons) : m_UseRibbons(useRibbons) {
    this->m_Start = std::move(start);
}

double Edge::computeTrueCost(const Map::SharedPtr& map, const DynamicObstaclesManager& obstacles,
                             double maxSpeed, double maxTurningRadius) {
    throw std::runtime_error("No longer implemented");
}

double Edge::computeApproxCost(double maxSpeed, double turningRadius) {
    if (start()->state().isCoLocated(end()->state())) {
        m_ApproxCost = 0;
    } else {
        m_DubinsWrapper.set(start()->state(), end()->state(), turningRadius);

        m_ApproxCost = m_DubinsWrapper.length() / maxSpeed * Edge::timePenalty();
    }
    return m_ApproxCost;
}

double Edge::netTime() {
    return end()->state().time() - start()->state().time();
}

void Edge::smooth(Map::SharedPtr map, const DynamicObstaclesManager& obstacles, double maxSpeed, double maxTurningRadius) {
    if (start()->isRoot()) return;
    double parentCost = start()->parentEdge()->trueCost(); // should be up to date in A*, check for BIT*
    auto smoothed = Vertex::connect(start()->parent(), end()->state());
    double smoothedCost = smoothed->parentEdge()->computeTrueCost(map, obstacles, maxSpeed, maxTurningRadius);
    if (smoothedCost < parentCost + m_TrueCost && smoothed->approxToGo() <= end()->approxToGo()) {
        // Should be memory-safe, as smoothed will delete the old vertex when it goes out of scope, and all pointers
        // to *end() will still be valid
        // TODO! -- apparently this is actually broken (something throws std::bad_weak_ptr when smoothing runs)
        std::swap(*(smoothed.get()), *(end().get()));
    }
}

DubinsWrapper Edge::getPlan(const PlannerConfig& config) {
    approxCost(); // throw the error if not calculated
    return m_DubinsWrapper; // TODO -- maybe visualize something here
//    DubinsWrapper d(start()->state(), end()->state(), start()->turningRadius());
//    Plan plan;
//    plan.append(d);
//    if (config.visualizations()) {
//        State s;
//        s.time() = start()->state().time();
//        for (; s.time() < end()->state().time(); s.time() += 1) {
//            d.sample(s);
//            config.visualizationStream() << "State: (" << s.toString() << "), f: " << 0 << ", g: " << 0 << ", h: " <<
//                                                                       0 << " plan" << std::endl;
//        }
//    }
//    return plan;
}

std::shared_ptr<Vertex> Edge::setEnd(const State &state) {
    auto ptr = std::make_shared<Vertex>(state, std::shared_ptr<Edge>(this), m_UseRibbons);
    m_End = ptr;
    return ptr;
}

std::shared_ptr<Vertex> Edge::start() const {
    return m_Start;
}

std::shared_ptr<Vertex> Edge::end() const {
    std::shared_ptr<Vertex> s(m_End);
    return s;
}

double Edge::trueCost() const {
    if (m_TrueCost == -1) throw std::logic_error("Fetching unset cached edge cost");
    if (m_TrueCost < 0 || !std::isfinite(m_TrueCost)) {
        std::cerr << "Invalid edge cost retrieved: " << m_TrueCost << " from edge between vertices\n\t" <<
            start()->toString() << " and\n\t" << end()->toString() << std::endl;
    }
    return m_TrueCost;
}

double Edge::approxCost() const {
    if (m_ApproxCost == -1) throw std::logic_error("Fetching unset cached approximate edge cost");
    return m_ApproxCost;
}

bool Edge::infeasible() const {
    return m_Infeasible;
}

double Edge::computeTrueCost(const Map::SharedPtr& map, const DynamicObstaclesManager& obstacles) {
    return computeTrueCost(map, obstacles, end()->state().speed(), end()->turningRadius());
}

double Edge::computeApproxCost() {
    return computeApproxCost(end()->state().speed(), end()->turningRadius());
}

double Edge::computeTrueCost(const PlannerConfig& config) {
    if (start()->state().isCoLocated(end()->state())) {
        std::cerr << "Computing cost of edge between two co-located states is likely an error" << std::endl;
    }
    double maxSpeed, maxTurningRadius;
    maxSpeed = config.maxSpeed();
    assert(maxSpeed > 0);
    if (end()->coverageAllowed()) {
        maxTurningRadius = config.coverageTurningRadius();
    } else {
        maxTurningRadius = config.turningRadius();
    }
    if (m_ApproxCost == -1) computeApproxCost(maxSpeed, maxTurningRadius);
    if (m_ApproxCost <= 0) throw std::runtime_error("Could not compute approximate cost");
    double collisionPenalty = 0;
    std::vector<std::pair<double, double>> result;
    State intermediate(start()->state());
    double length = m_DubinsWrapper.length();
    // truncate longer edges than 30 seconds // TODO! -- use actual horizon variable
    auto remainingTime = 30 + 1 + config.startStateTime() - start()->state().time();
    if (length > maxSpeed * remainingTime) length = maxSpeed * remainingTime;

    double dynamicDistance = 0, toCoverDistance = 0;
    std::vector<std::pair<double, double>> newlyCovered;
    double lastHeading = start()->state().heading();
    int visCount = int(1.0 / Edge::dubinsIncrement()); // counter to reduce visualization frequency

    auto startG = start()->currentCost();
    auto startH = start()->approxToGo();

    // collision check along the curve (and watch out for newly covered points, too)
    while (intermediate.time() <= remainingTime) {
        m_DubinsWrapper.sample(intermediate);
        // visualize
        if (config.visualizations() && visCount-- <= 0) {
            visCount = int(1.0 / Edge::dubinsIncrement());
            auto timeSoFar = intermediate.time() - start()->state().time();
            auto gSoFar = startG + timeSoFar + collisionPenalty;
            // should really put visualizeVertex somewhere accessible
            // use start H because it isn't worth it to calculate current H
            config.visualizationStream() << "State: (" << intermediate.toString() << "), f: " << gSoFar + startH <<
                ", g: " << gSoFar << ", h: " << startH << " trajectory" << std::endl;
        }
        if (config.map()->getUnblockedDistance(intermediate.x(), intermediate.y()) <= Edge::dubinsIncrement()) {
            collisionPenalty += Edge::collisionPenalty();
            std::cerr << "Infeasible edge discovered" << std::endl;
            m_Infeasible = true;
            break;
        }
        if (dynamicDistance > Edge::dubinsIncrement()) {
            dynamicDistance -= Edge::dubinsIncrement();
        } else {
            dynamicDistance = config.obstacles().distanceToNearestPossibleCollision(intermediate);
            if (dynamicDistance <= Edge::dubinsIncrement()) {
                assert(std::isfinite(maxSpeed));
                assert(std::isfinite(config.obstacles().collisionExists(intermediate)));
                collisionPenalty += config.obstacles().collisionExists(intermediate) * Edge::collisionPenalty();
                dynamicDistance = 0;
            }
        }
        if (toCoverDistance > Edge::dubinsIncrement()) {
            toCoverDistance -= Edge::dubinsIncrement();
        } else {
            if (m_UseRibbons) {
                // do this first because cover splits ribbons so you'd never get one that "contains" the point so it
                // could be a bit more work
                toCoverDistance = end()->ribbonManager().minDistanceFrom(intermediate.x(), intermediate.y());
                if (end()->coverageAllowed() || lastHeading == intermediate.yaw()) {
                    end()->ribbonManager().cover(intermediate.x(), intermediate.y());
                }
            } else {
                toCoverDistance = DBL_MAX;
                for (auto p : start()->uncovered().get()) {
                    auto d = Path::distance(p, intermediate.x(), intermediate.y());
                    if (Path::covers(d)) {
                        newlyCovered.push_back(p);
                    } else {
                        toCoverDistance = fmin(toCoverDistance, d - Path::coverageThreshold());
                    }
                }
            }
        }
        intermediate.time() += Edge::dubinsIncrement() / maxSpeed;
        lastHeading = intermediate.heading();
    }
    // make sure we're close // not valid because we truncate if too long
//    assert(fabs(end()->state().x - q[0]) < Edge::dubinsIncrement() &&
//        fabs(end()->state().y - q[1]) < Edge::dubinsIncrement());
    // set state to be at the end of where we collision checked
    end()->state().x() = intermediate.x(); end()->state().y() = intermediate.y(); end()->state().yaw(intermediate.yaw());


    // set end's state's time
    end()->state().time() = start()->state().time() + length / maxSpeed;

    if (!m_UseRibbons) {
        // remove duplicates for efficiency for the next bit
        std::sort(newlyCovered.begin(), newlyCovered.end());
        newlyCovered.erase(std::unique(newlyCovered.begin(), newlyCovered.end()), newlyCovered.end());

        // set end's uncovered list
        end()->uncovered().clear();
        for (auto p : start()->uncovered().get()) {
            if (std::find(newlyCovered.begin(), newlyCovered.end(), p) == newlyCovered.end()) {
                end()->uncovered().add(p);
            }
        }
    }
    assert(std::isfinite(netTime()));
    assert(std::isfinite(collisionPenalty));
    m_CollisionPenalty = collisionPenalty;
    m_TrueCost = netTime() * Edge::timePenalty() + collisionPenalty;

    end()->setCurrentCost();

    return m_TrueCost;
}

void Edge::computeBrownPath(const PlannerConfig& config, const Ribbon& r) {

    // I wrote this in the RibbonManager first and copied it here, so that's why this is a little awkward
    auto start = m_Start->state();
    auto radius = config.coverageTurningRadius();

    auto h = start.yaw() + M_PI_2;
    // Get points one radius away from start in the directions perpendicular to its heading.
    // These are centers of circles on which start is tangent
    auto x1 = start.x() + cos(h) * radius;
    auto x2 = start.x() - cos(h) * radius;
    auto y1 = start.y() + sin(h) * radius;
    auto y2 = start.y() - sin(h) * radius;

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
    auto h2 = s.yaw() - M_PI_2;
    auto dx1 = cos(h2) * radius / 2;
    auto dy1 = sin(h2) * radius / 2;
    auto x3 = proj.first + proj.first < x ? dx1 : -dx1;
    auto y3 = proj.second = proj.second < y ? dy1 : -dy1;
//        auto x4 = proj2.first = proj2.first < x2 ? dx1 : -dx1;
//        auto y4 = proj2.second = proj2.second < y2 ? dy1 : -dy1;

    // extend that ahead until it reaches the circle around (x1, y1)
    // sqrt(a^2 - r^2)
    auto a = dx1 * dx1 + dy1 * dy1;
    auto rSquared = radius * radius;
    auto b = sqrt(rSquared - a);
    auto h3 = s.yaw();
    auto x5 = x3 + b * cos(h3);
    auto y5 = x3 + b * sin(h3);
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
    State final(projFinal.first, projFinal.second, s.heading(), 0, 0);

    // done. that's the state.
    end()->state() = final;

    // now to figure out a Dubins curve for it

}


Edge::~Edge() = default;
