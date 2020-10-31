#include "Edge.h"
#include <path_planner_common/State.h>

#include <utility>
#include <memory>

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
    return m_DubinsWrapper;
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

double Edge::computeTrueCost(PlannerConfig& config) {
    if (start()->state().isCoLocated(end()->state())) {
        std::cerr << "Computing cost of edge between two co-located states is likely an error" << std::endl;
    }
    // get speed from end state
    double speed = end()->state().speed(), turningRadius = config.turningRadius();
    assert(speed > 0);
    if (end()->coverageAllowed()) {
        turningRadius = config.coverageTurningRadius();
    }
    if (m_ApproxCost == -1 || (m_DubinsWrapper.getRho() != turningRadius))
        // if the parameters are different now we need to re-calculate the curve
        computeApproxCost(speed, turningRadius);
    if (m_DubinsWrapper.getSpeed() != speed) {
        // update the speed if it's different
        m_DubinsWrapper.setSpeed(speed);
    }
    if (m_ApproxCost < 0) throw std::runtime_error("Could not compute approximate cost");
    double collisionPenalty = 0;
    std::vector<std::pair<double, double>> result;
    State intermediate(start()->state());
    // truncate longer edges than 30 seconds
    auto endTime = fmin(config.timeHorizon() + 1e-12 + config.startStateTime(),m_DubinsWrapper.getEndTime());
    // time, relative to this edge, of when the ribbons are done (not super necessary but convenient)
    auto ribbonsDoneTime = -1;
    auto ribbonManagerStartedDone = end()->ribbonManager().done();

    double dynamicDistance = 0, toCoverDistance = 0;
    double lastHeading = start()->state().heading();
    int visCount = int(1.0 / config.collisionCheckingIncrement()); // counter to reduce visualization frequency

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

    // Collision check at max speed, even if we're going slower. This will make going slower in congested areas look
    // artificially better, because they'll accrue a smaller penalty per time
    auto timeIncrement = config.collisionCheckingIncrement() / config.maxSpeed();

    // nudge along a little so we check at an even amount of intervals from the start state
    // this helps make sure every plan is scored equally
    auto timeSinceStart = intermediate.time() - config.startStateTime();
    auto timeNudge = fmod(timeSinceStart, timeIncrement);
    intermediate.time() += timeNudge;

    if (config.visualizations())
        config.visualizationStream() << "Trajectory:" << std::endl;
    // collision and coverage check along the curve
    while (intermediate.time() < endTime) {
        try {
            m_DubinsWrapper.sample(intermediate);
        }
        catch (std::runtime_error& e) {
            m_Infeasible = true;
            *config.output() << "Encountered an error while collision checking: " << e.what() << std::endl;
            break;
        }
        // visualize
        if (config.visualizations() && visCount-- <= 0) {
            visCount = int(1.0 / config.collisionCheckingIncrement());
            auto timeSoFar = intermediate.time() - start()->state().time();
            auto gSoFar = startG + timeSoFar + collisionPenalty;
            // should really put visualizeVertex somewhere accessible
            // use start H because it isn't worth it to calculate current H
            config.visualizationStream() << "State: (" << intermediate.toStringRad() << "), f: " << gSoFar + startH <<
                ", g: " << gSoFar << ", h: " << startH << " trajectory" << std::endl;
        }
        if (config.map()->isBlocked(intermediate.x(), intermediate.y())) {
            m_Infeasible = true;
            break;
        }

        // assess collision penalty
        collisionPenalty +=
                config.obstaclesManager().collisionExists(intermediate, true) * Edge::collisionPenaltyFactor();

        if (toCoverDistance > config.collisionCheckingIncrement()) {
            toCoverDistance -= config.collisionCheckingIncrement();
        } else {
            // do this first because cover splits ribbons so you'd never get one that "contains" the point so it
            // could be a bit more work
            toCoverDistance = end()->ribbonManager().minDistanceFrom(intermediate.x(), intermediate.y());
            if (end()->coverageAllowed() || lastHeading == intermediate.heading()) {
                end()->ribbonManager().cover(intermediate.x(), intermediate.y(), true);
            }
            if (end()->ribbonManager().done()) {
                // if no prior edge has finished coverage yet, set the coverage completed time now
                if (end()->ribbonManager().coverageCompletedTime() == -1) {
                    end()->ribbonManager().setCoverageCompletedTime(intermediate.time());
                }
                ribbonsDoneTime = intermediate.time();
                // truncate only if we hit the time minimum *after coverage* - the adjusted end time
                endTime = fmin(endTime, end()->ribbonManager().coverageCompletedTime() + config.timeMinimum());
            }

        }
        intermediate.time() += timeIncrement;
        lastHeading = intermediate.heading();
    }
    // set to the end of the edge (potentially truncated)
    end()->state().time() = endTime;
    m_DubinsWrapper.sample(end()->state());
    m_DubinsWrapper.updateEndTime(end()->state().time()); // should just be truncating the path

    // cover the last little bit
    if (end()->coverageAllowed() || lastHeading == intermediate.heading()) {
        end()->ribbonManager().cover(intermediate.x(), intermediate.y(), true);
    }
    if (end()->ribbonManager().done()) {
        // may need to set the time here too
        if (end()->ribbonManager().coverageCompletedTime() == -1) {
            end()->ribbonManager().setCoverageCompletedTime(intermediate.time());
        }
        ribbonsDoneTime = intermediate.time();
    }

    assert(std::isfinite(netTime()));
    assert(std::isfinite(collisionPenalty));
    m_CollisionPenalty = collisionPenalty;
    // time after ribbons covered doesn't count against you
    auto t = fmax(netTime() - (end()->ribbonManager().done()? (endTime - ribbonsDoneTime) : 0), 0);
    if (ribbonManagerStartedDone) t = 0;
    m_TrueCost = t * Edge::timePenaltyFactor() + collisionPenalty;

    end()->setCurrentCost();

    end()->computeApproxToGo(config);

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
