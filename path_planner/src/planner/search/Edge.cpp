#include <utility>
#include <algorithm>
#include <memory>
#include "Edge.h"
#include <cfloat>

Edge::Edge(std::shared_ptr<Vertex> start) {
    this->m_Start = std::move(start);
}

double Edge::computeApproxCost(double maxSpeed, double turningRadius) {
    if (start()->state().isCoLocated(end()->state())) {
        m_ApproxCost = 0;
    } else {
        m_DubinsWrapper.set(start()->state(), end()->state(), turningRadius);

        m_ApproxCost = m_DubinsWrapper.length() / maxSpeed * Edge::timePenaltyFactor();
    }
    return m_ApproxCost;
}

double Edge::netTime() {
    return end()->state().time() - start()->state().time();
}

DubinsWrapper Edge::getPlan(const PlannerConfig& config) {
    approxCost(); // throw the error if not calculated
    return m_DubinsWrapper; // TODO -- maybe visualize something here
}

std::shared_ptr<Vertex> Edge::setEnd(const State &state) {
    auto ptr = std::make_shared<Vertex>(state, std::shared_ptr<Edge>(this));
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

double Edge::computeApproxCost() {
    return computeApproxCost(end()->state().speed(), end()->turningRadius());
}

double Edge::computeTrueCost(const PlannerConfig& config) {
    if (start()->state().isCoLocated(end()->state())) {
        std::cerr << "Computing cost of edge between two co-located states is likely an error" << std::endl;
    }
    double speed = config.maxSpeed(), turningRadius = config.turningRadius();
    assert(speed > 0);
    if (end()->coverageAllowed()) {
        turningRadius = config.coverageTurningRadius();
    }
    if (m_ApproxCost == -1 || (m_DubinsWrapper.getRho() != turningRadius || m_DubinsWrapper.getSpeed() != speed))
        // if the parameters are different now we need to re-calculate the curve
        computeApproxCost(speed, turningRadius);
    if (m_ApproxCost < 0) throw std::runtime_error("Could not compute approximate cost");
    double collisionPenalty = 0;
    std::vector<std::pair<double, double>> result;
    State intermediate(start()->state());
    // truncate longer edges than 30 seconds
    auto endTime = fmin(DubinsPlan::timeHorizon() + 1 + config.startStateTime(),m_DubinsWrapper.getEndTime());
    auto minGoalTime = fmin(m_DubinsWrapper.getEndTime(), DubinsPlan::timeMinimum() + config.startStateTime() + 1e-5);

    double dynamicDistance = 0, toCoverDistance = 0;
    std::vector<std::pair<double, double>> newlyCovered;
    double lastHeading = start()->state().heading();
    int visCount = int(1.0 / Edge::collisionCheckingIncrement()); // counter to reduce visualization frequency

    auto startG = start()->currentCost();
    auto startH = start()->approxToGo();

    if (intermediate.time() >= endTime) {
        if (intermediate.time() > endTime) std::cerr << "Negative length edge" << std::endl;
        else {
            std::cerr << "Zero length edge: " << std::endl;
            std::cerr << "\t" << start()->state().toString() << std::endl;
            std::cerr << "\t" << end()->state().toString() << std::endl;
        }
        m_Infeasible = true;
    }
    if (config.visualizations())
        config.visualizationStream() << "Trajectory:" << std::endl;
    // collision check along the curve (and watch out for newly covered points, too)
    while (intermediate.time() < endTime) {
        m_DubinsWrapper.sample(intermediate);
        // visualize
        if (config.visualizations() && visCount-- <= 0) {
            visCount = int(1.0 / Edge::collisionCheckingIncrement());
            auto timeSoFar = intermediate.time() - start()->state().time();
            auto gSoFar = startG + timeSoFar + collisionPenalty;
            // should really put visualizeVertex somewhere accessible
            // use start H because it isn't worth it to calculate current H
            config.visualizationStream() << "State: (" << intermediate.toStringRad() << "), f: " << gSoFar + startH <<
                ", g: " << gSoFar << ", h: " << startH << " trajectory" << std::endl;
        }
        if (config.map()->getUnblockedDistance(intermediate.x(), intermediate.y()) <= Edge::collisionCheckingIncrement()) {
            collisionPenalty += Edge::collisionPenaltyFactor();
            std::cerr << "Infeasible edge discovered" << std::endl;
            m_Infeasible = true;
            break;
        }
        if (dynamicDistance > Edge::collisionCheckingIncrement()) {
            dynamicDistance -= Edge::collisionCheckingIncrement();
        } else {
            dynamicDistance = config.obstacles().distanceToNearestPossibleCollision(intermediate);
            if (dynamicDistance <= Edge::collisionCheckingIncrement()) {
                assert(std::isfinite(speed));
                assert(std::isfinite(config.obstacles().collisionExists(intermediate)));
                collisionPenalty += config.obstacles().collisionExists(intermediate) * Edge::collisionPenaltyFactor();
                dynamicDistance = 0;
            }
        }
        if (toCoverDistance > Edge::collisionCheckingIncrement()) {
            toCoverDistance -= Edge::collisionCheckingIncrement();
        } else {
            // do this first because cover splits ribbons so you'd never get one that "contains" the point so it
            // could be a bit more work
            toCoverDistance = end()->ribbonManager().minDistanceFrom(intermediate.x(), intermediate.y());
            if (end()->coverageAllowed() || lastHeading == intermediate.heading()) {
                end()->ribbonManager().cover(intermediate.x(), intermediate.y());
            }
            if (end()->ribbonManager().done()) {
                // make it so we only need the min time if we finish covering
                endTime = fmax(minGoalTime, intermediate.time());
            }

        }
        intermediate.time() += Edge::collisionCheckingIncrement() / speed;
        lastHeading = intermediate.heading();
    }
    // set to the end of the edge (potentially truncated)
    end()->state().time() = endTime;
    m_DubinsWrapper.sample(end()->state());
    m_DubinsWrapper.updateEndTime(end()->state().time()); // should just be truncating the path

    assert(std::isfinite(netTime()));
    assert(std::isfinite(collisionPenalty));
    m_CollisionPenalty = collisionPenalty;
    m_TrueCost = netTime() * Edge::timePenaltyFactor() + collisionPenalty;

    end()->setCurrentCost();

    return m_TrueCost;
}

std::shared_ptr<Vertex> Edge::setEnd(const DubinsWrapper& path) {
    m_DubinsWrapper = path;
    State s;
    s.time() = path.getEndTime();
    path.sample(s);
    m_ApproxCost = (s.time() - start()->state().time()) * timePenaltyFactor();
    return setEnd(s);
}


Edge::~Edge() = default;
