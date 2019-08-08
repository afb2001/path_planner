#include <utility>

#include <utility>

#include <utility>
#include <algorithm>
#include <memory>
#include "Edge.h"
#include "../common/Map.h"
#include "../common/dynamic_obstacles/DynamicObstaclesManager.h"
#include "../common/Path.h"
#include <robust_dubins/RobustDubins.h>
#include <cfloat>

extern "C" {
#include <dubins.h>
}

Edge::Edge(std::shared_ptr<Vertex> start) {
    this->m_Start = std::move(start);
}

Edge::Edge(std::shared_ptr<Vertex> start, const State& end) : Edge(std::move(start)){
    setEnd(end);
}

//double Edge::computeTrueCost(Map *map, DynamicObstaclesManager *obstacles,
//                             double maxSpeed, double maxTurningRadius) {
//    double collisionPenalty = 0;
//    if (m_ApproxCost == -1) computeApproxCost(maxSpeed, maxTurningRadius);
//    if (m_ApproxCost == 0) {
//        m_TrueCost = 0; // edge is a point
//        return 0;
//    }
//    double& penalty = collisionPenalty;
//    std::vector<std::pair<double, double>> result;
//    std::vector<double> x,y,h; //
//    dubinsPath.computePathHistory();
//    x = dubinsPath.get_xHistory();
//    y = dubinsPath.get_yHistory();
//    double skipDistance = 0, lengthSoFar = 0;
//    std::vector<std::pair<double, double>> newlyCovered;
//    for (int i = 0; i < x.size(); i++) {
//        if (skipDistance > DUBINS_INCREMENT) {
//            skipDistance -= DUBINS_INCREMENT;
//        } else {
//            double staticDistance = map->getUnblockedDistance(x[i], y[i]);
//            if (staticDistance <= 0) {
//                penalty += COLLISION_PENALTY;
//                skipDistance = 0;
//            } else {
//                // TODO! -- the new Dubins library doesn't make promises about spacing so the times could be off here
//                double dynamicDistance = obstacles->distanceToNearestPossibleCollision(x[i], y[i], maxSpeed,
//                                                                                       start()->state().time +
//                                                                                       lengthSoFar / maxSpeed);
//                if (dynamicDistance <= 0) {
//                    penalty += obstacles->collisionExists(x[i], y[i], start()->state().time + lengthSoFar / maxSpeed) * COLLISION_PENALTY;
//                    skipDistance = 0;
//                } else {
//                    skipDistance = fmin(staticDistance, dynamicDistance);
//                }
//            }
//        }
//        // TODO! -- min distance to uncovered point
//        for (auto p : start()->uncovered().get()) {
//            if (Path::covers(p, x[i], y[i])) {
//                newlyCovered.push_back(p);
//            }
//        }
//        lengthSoFar += DUBINS_INCREMENT;
//    }
//
//    if (!newlyCovered.empty()) {
//        std::sort(newlyCovered.begin(), newlyCovered.end());
//        newlyCovered.erase(std::unique(newlyCovered.begin(), newlyCovered.end()), newlyCovered.end());
//    }
//
//    // set end's state's time
//    end()->state().time = start()->state().time + dubinsPath.get_cost() / maxSpeed;
//
//    // set end's uncovered list
//    end()->uncovered().clear();
//    for (auto p : start()->uncovered().get()) {
//        if (std::find(newlyCovered.begin(), newlyCovered.end(), p) == newlyCovered.end()) {
//            end()->uncovered().add(p);
//        }
//    }
//    m_TrueCost = netTime() * TIME_PENALTY + collisionPenalty;
//
//    end()->setCurrentCost();
//
//    return m_TrueCost;
//}

double Edge::computeTrueCost(Map *map, DynamicObstaclesManager *obstacles,
                             double maxSpeed, double maxTurningRadius) {
    double collisionPenalty = 0;
    if (m_ApproxCost == -1) computeApproxCost(maxSpeed, maxTurningRadius);
    double& penalty = collisionPenalty;
    std::vector<std::pair<double, double>> result;
    double q[3];
    double lengthSoFar = 0;
    double length = dubins_path_length(&dubinsPath);
    double staticDistance = 0, dynamicDistance = 0, toCoverDistance = 0;
    std::vector<std::pair<double, double>> newlyCovered;

    // collision check along the curve (and watch out for newly covered points, too)
    while (lengthSoFar < length) {
        dubins_path_sample(&dubinsPath, lengthSoFar, q);
        if (staticDistance > DUBINS_INCREMENT) {
            staticDistance -= DUBINS_INCREMENT;
        } else {
            staticDistance = map->getUnblockedDistance(q[0], q[1]);
            if (staticDistance <= DUBINS_INCREMENT) {
                penalty += COLLISION_PENALTY;
                staticDistance = 0;
            }
        }
        if (dynamicDistance > DUBINS_INCREMENT) {
            dynamicDistance -= DUBINS_INCREMENT;
        } else {
            dynamicDistance = obstacles->distanceToNearestPossibleCollision(q[0], q[1], start()->state().speed,
                                                                            start()->state().time +
                                                                            (lengthSoFar / maxSpeed));
            if (dynamicDistance <= DUBINS_INCREMENT) {
                penalty += obstacles->collisionExists(q[0], q[1], start()->state().time + (lengthSoFar / maxSpeed)) *
                           COLLISION_PENALTY;
                dynamicDistance = 0;
            }
        }
        if (toCoverDistance > DUBINS_INCREMENT) {
            toCoverDistance -= DUBINS_INCREMENT;
        } else {
            toCoverDistance = DBL_MAX;
            for (auto p : start()->uncovered().get()) {
                auto d = Path::distance(p, q[0], q[1]);
                if (Path::covers(d)) {
                    newlyCovered.push_back(p);
                } else {
                    toCoverDistance = fmin(toCoverDistance, d);
                }
            }
        }
        lengthSoFar += DUBINS_INCREMENT;
    }
    if (!newlyCovered.empty()) {
        std::sort(newlyCovered.begin(), newlyCovered.end());
        newlyCovered.erase(std::unique(newlyCovered.begin(), newlyCovered.end()), newlyCovered.end());
    }

    // set end's state's time
    end()->state().time = start()->state().time + dubins_path_length(&dubinsPath) / maxSpeed;

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

//double Edge::computeApproxCost(double maxSpeed, double maxTurningRadius) {
//    if (start()->state().colocated(end()->state())) {
//        m_ApproxCost = 0;
//    } else {
//        RobustDubins::Problem problem;
//        problem.set_stateInitial(start()->state().x, start()->state().y, start()->state().yaw());
//        problem.set_stateFinal(end()->state().x, end()->state().y, end()->state().yaw());
//        problem.set_minTurningRadius(maxTurningRadius);
//        RobustDubins::Solver solver;
//        solver.set_problemStatement(problem);
//        solver.solve();
//        dubinsPath = solver.get_optimalPath();
//        dubinsPath.set_spacing(DUBINS_INCREMENT); // do this now for computing true cost and getting a plan
//        m_ApproxCost = dubinsPath.get_cost() / maxSpeed * TIME_PENALTY;
//    }
//    return m_ApproxCost;
//}

double Edge::computeApproxCost(double maxSpeed, double maxTurningRadius) {
    if (start()->state().colocated(end()->state())) {
        m_ApproxCost = 0;
    } else {
        double q0[3] = {start()->state().x, start()->state().y, start()->state().yaw()};
        double q1[3] = {end()->state().x, end()->state().y, end()->state().yaw()};
        int err = dubins_shortest_path(&dubinsPath, q0, q1, maxTurningRadius);
        if (err != EDUBOK) {
            std::cerr << "Encountered an error in the Dubins library" << std::endl;
        } else {
            m_ApproxCost = dubins_path_length(&dubinsPath) / maxSpeed * TIME_PENALTY;
        }
    }
    return m_ApproxCost;
}

double Edge::netTime() {
    return end()->state().time - start()->state().time;
}

void Edge::smooth(Map* map, DynamicObstaclesManager* obstacles, double maxSpeed, double maxTurningRadius) {
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

//Plan Edge::getPlan(double maxSpeed) {
//    double lengthSoFar = 0;
//    Plan plan;
//    std::vector<double> x,y,h; //
//    dubinsPath.computePathHistory();
//    x = dubinsPath.get_xHistory();
//    y = dubinsPath.get_yHistory();
//    h = dubinsPath.get_hHistory();
//    for (int i = 0; i < x.size(); i++) {
//        plan.append(State(x[i], y[i], M_PI_2 - h[i], maxSpeed, lengthSoFar / maxSpeed + start()->state().time));
//        lengthSoFar += DUBINS_INCREMENT;
//    }
//    return plan;
//}

Plan Edge::getPlan(double maxSpeed) {
    double q[3];
    double lengthSoFar = 0;
    double length = dubins_path_length(&dubinsPath);
    Plan plan1;
    while (lengthSoFar < length) {
        dubins_path_sample(&dubinsPath, lengthSoFar, q);
        plan1.append(State(q[0], q[1], M_PI_2 - q[2], maxSpeed, lengthSoFar / maxSpeed + start()->state().time));
        lengthSoFar += DUBINS_INCREMENT;
    }
    return plan1;
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
    if (m_TrueCost == -1) throw std::logic_error("Fetching unset cached edge cost");
    return m_TrueCost;
}

double Edge::approxCost() const {
    if (m_ApproxCost == -1) throw std::logic_error("Fetching unset cached approximate edge cost");
    return m_ApproxCost;
}

Edge::~Edge() = default;
