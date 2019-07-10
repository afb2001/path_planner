#include <utility>

#include <utility>

#include <utility>
#include <algorithm>
#include <memory>
#include "Edge.h"
#include "../common/Map.h"
#include "../common/DynamicObstacles.h"
#include "../common/Path.h"
#include <robust_dubins/RobustDubins.h>

//extern "C" {
//#include <dubins.h>
//}

Edge::Edge(std::shared_ptr<Vertex> start) {
    this->m_Start = std::move(start);
}

Edge::Edge(std::shared_ptr<Vertex> start, const State& end) : Edge(std::move(start)){
    setEnd(end);
}

double Edge::computeTrueCost(Map *map, DynamicObstacles *obstacles,
                             double maxSpeed, double maxTurningRadius) {
    double collisionPenalty;
    if (m_ApproxCost == -1) computeApproxCost(maxSpeed, maxTurningRadius);
    if (m_ApproxCost == 0) {
        m_TrueCost = 0; // edge is a point
        return 0;
    }
    double& penalty = collisionPenalty;
    std::vector<std::pair<double, double>> result;
    std::vector<double> x,y,h; //
    dubinsPath.computePathHistory();
    x = dubinsPath.get_xHistory();
    y = dubinsPath.get_yHistory();
    double obstacleDistance = 0, lengthSoFar = 0;
    std::vector<std::pair<double, double>> newlyCovered;
    for (int i = 0; i < x.size(); i++) {
        if (obstacleDistance > DUBINS_INCREMENT) {
            obstacleDistance -= DUBINS_INCREMENT;
        } else {
            double staticDistance = map->getUnblockedDistance(x[i], y[i]);
            if (staticDistance <= 0) {
                penalty += COLLISION_PENALTY;
                obstacleDistance = 0;
            } else {
                // TODO! -- the new Dubins library doesn't make promises about spacing so the times could be off here
                double dynamicDistance = obstacles->distanceToNearestPossibleCollision(x[i], y[i], maxSpeed,
                                                                                       start()->state().time +
                                                                                       lengthSoFar / maxSpeed);
                if (dynamicDistance <= 0) {
                    penalty += obstacles->collisionExists(x[i], y[i], start()->state().time + lengthSoFar / maxSpeed) * COLLISION_PENALTY;
                    obstacleDistance = 0;
                } else {
                    obstacleDistance = fmin(staticDistance, dynamicDistance);
                }
            }
        }
        for (auto p : start()->uncovered().get()) {
            if ((p.first - x[i]) * (p.first - x[i]) + (p.second - y[i]) * (p.second - y[i]) < COVERAGE_THRESHOLD * COVERAGE_THRESHOLD) {
                newlyCovered.push_back(p);
            }
        }
        lengthSoFar += DUBINS_INCREMENT;
    }

    if (!newlyCovered.empty()) {
        std::sort(newlyCovered.begin(), newlyCovered.end());
        newlyCovered.erase(std::unique(newlyCovered.begin(), newlyCovered.end()), newlyCovered.end());
    }

    // set end's state's time
    end()->state().time = start()->state().time + dubinsPath.get_cost() / maxSpeed;

    // set end's uncovered list
    end()->uncovered().clear();
    for (auto p : start()->uncovered().get()) {
        if (std::find(newlyCovered.begin(), newlyCovered.end(), p) == newlyCovered.end()) {
            end()->uncovered().add(p);
        }
    }
    m_TrueCost = netTime() * TIME_PENALTY + collisionPenalty;

    end()->setCurrentCost();

    return m_TrueCost;
}

double Edge::computeApproxCost(double maxSpeed, double maxTurningRadius) {
    if (start()->state().colocated(end()->state())) {
        m_ApproxCost = 0;
    } else {
        RobustDubins::Problem problem;
        problem.set_stateInitial(start()->state().x, start()->state().y, start()->state().yaw());
        problem.set_stateFinal(end()->state().x, end()->state().y, end()->state().yaw());
        problem.set_minTurningRadius(maxTurningRadius);
        RobustDubins::Solver solver;
        solver.set_problemStatement(problem);
        solver.solve();
        dubinsPath = solver.get_optimalPath();
        dubinsPath.set_spacing(DUBINS_INCREMENT); // do this now for computing true cost and getting a plan
        m_ApproxCost = dubinsPath.get_cost() / maxSpeed * TIME_PENALTY;
    }
    return m_ApproxCost;
}

double Edge::netTime() {
    return end()->state().time - start()->state().time;
}

void Edge::smooth(Map* map, DynamicObstacles* obstacles, double maxSpeed, double maxTurningRadius) {
    if (start()->isRoot()) return;
    double parentCost = start()->parentEdge()->m_TrueCost; // should be up to date in A*, check for BIT*
    auto smoothed = Vertex::connect(start()->parent(), end()->state());
    double smoothedCost = smoothed->parentEdge()->computeTrueCost(map, obstacles, maxSpeed, maxTurningRadius);
    if (smoothedCost < parentCost + m_TrueCost && smoothed->approxToGo() <= end()->approxToGo()) {
        // Should be memory-safe, as smoothed will delete the old vertex when it goes out of scope, and all pointers
        // to *end() will still be valid
        // TODO! -- apparently this is actually broken (something throws std::bad_weak_ptr when smoothing runs)
        std::swap(*(smoothed.get()), *(end().get()));
    }
}

Plan Edge::getPlan(double maxSpeed) {
    double lengthSoFar = 0;
    Plan plan;
    std::vector<double> x,y,h; //
    dubinsPath.computePathHistory();
    x = dubinsPath.get_xHistory();
    y = dubinsPath.get_yHistory();
    h = dubinsPath.get_hHistory();
    for (int i = 0; i < x.size(); i++) {
        plan.append(State(x[i], y[i], M_PI_2 - h[i], maxSpeed, lengthSoFar / maxSpeed + start()->state().time));
        lengthSoFar += DUBINS_INCREMENT;
    }
    return plan;
}

std::shared_ptr<Vertex> Edge::setEnd(const State &state) {
    auto ptr = std::make_shared<Vertex>(state, std::shared_ptr<Edge>(this));
    m_End = ptr;
    return ptr;
}

std::shared_ptr<Vertex> Edge::start() {
    return m_Start;
}

std::shared_ptr<Vertex> Edge::end() {
    std::shared_ptr<Vertex> s(m_End);
    return s;
}

double Edge::trueCost() const {
    return m_TrueCost;
}

Edge::~Edge() = default;
