#include "PotentialFieldsPlanner.h"
#include "../common/dynamic_obstacles/BinaryDynamicObstaclesManager.h"

Planner::Stats PotentialFieldsPlanner::plan(const RibbonManager& ribbonManager, const State& start,
                                            PlannerConfig config, const DubinsPlan& previousPlan,
                                            double timeRemaining) {
    Stats stats;
    auto current = start;
    current.speed() = config.maxSpeed();
    auto localRibbonManager = ribbonManager;
    {
        // I wanna use the same variable name twice sue me
        auto aheadState = current.push(1);
        localRibbonManager.coverBetween(current.x(), current.y(), aheadState.x(), aheadState.y(), false);
    }
//    double nextDir = FP_NAN;

    // can't get onto a line reliably but once on we're pretty okay
    for (int i = 0; i < c_LookaheadSteps; i++) {
        const auto& ribbons = localRibbonManager.get();
        Force net(0, 0);
//        if (nextDir == FP_NAN) {
            for (const auto& r : ribbons) {
                // get ribbon endpoints and distances
                auto s = r.startAsState();
                auto e = r.endAsState();
                auto ds = current.distanceTo(s);
                bool startClose;
                auto de = current.distanceTo(e);
                bool endClose;

                // get the point 10m before the ribbon starts
                s.move(-10);
                auto dsMoved = current.distanceTo(s);
                if (dsMoved > 8 && ds > 3) {
                    // Case 1: we're far away from both the actual start and the point 10m before it
                    // Desired behavior: drive towards the point 10m before the ribbon start
                    startClose = false;
//                    ds = dsMoved;
                } else {
                    // Case 2: we're close enough to the ribbon start trying to go along it
                    // Desired behavior: drive towards the end point of the ribbon
                    startClose = true;
                    s = r.startAsState(); // not necessary?
                }

                e.move(-10);
                auto deMoved = current.distanceTo(e);
                // equivalent of above cases
                if (deMoved > 8 && de > 3) {
                    endClose = false;
//                    de = dsMoved;
                } else {
                    endClose = true;
                    e = r.endAsState();
                }

                State closest;
                // figure out which end to point to based on the above
                if (ds < de) {
                    if (startClose) {
                        // (Case 2)
                        closest = e;
                    } else {
                        // (Case 1)
                        closest = s;
                    }
                } else {
                    if (endClose) {
                        // (Case 2)
                        closest = s;
                    } else {
                        // (Case 1)
                        closest = e;
                    }
                }
                auto dClosest = fmin(ds, de);
                auto dirClosest = M_PI_2 - current.headingTo(closest);
                net = net + Force(getRibbonMagnitude(dClosest), dirClosest);
            }
            // TODO! -- repulsive forces
            // for static map, maybe load it again here and do brushfire? maybe make map loader do it and store dir?

            // or, instead, just query at the resolution in the map at all relevant distances
            auto resolution = config.map()->resolution();
            if (resolution > 0) {
                for (double x = current.x() - c_StaticObsIgnoreThreshold; x <= current.x() + c_StaticObsIgnoreThreshold; x += resolution) {
                    for (double y = current.y() - c_StaticObsIgnoreThreshold; y <= current.y() + c_StaticObsIgnoreThreshold; y += resolution) {
                        if (config.map()->isBlocked(x, y)) {
                            net = net - Force(getStaticObstacleMagnitude(current.distanceTo(x, y)),
                                              M_PI_2 - current.headingTo(x, y));
                        }
                    }
                }
            }
            // for dynamic obstacles let's maybe cast it to a binary obstacles manager and use the method for display?
            try {
                const auto& obstaclesManager = dynamic_cast<const BinaryDynamicObstaclesManager&>(config.obstaclesManager());
                for (const auto& o : obstaclesManager.get()) {
                    auto obstacle = o.second;
                    obstacle.project(current.time());
                    auto d = current.distanceTo(obstacle.X, obstacle.Y);
                    auto dir = M_PI_2 - current.headingTo(obstacle.X, obstacle.Y);
                    net = net - Force(getDynamicObstacleMagnitude(d, obstacle.Width, obstacle.Length), dir);
                }
            }
            catch (std::bad_cast&) {
                *m_Config.output() << "Warning: cannot use unknown dynamic obstacles manager with potential fields planner" << std::endl;
            }

//        }

        auto direction = net.getDirection();
        // hardcode the next direction if we set it
//        if (nextDir != FP_NAN) {
//            direction = nextDir;
//            nextDir = FP_NAN;
//        }
        current.setYaw(direction);
        auto s = current;
        // if magnitude is zero stay put
        if (net.Y != 0 || net.X != 0) {
            current = current.push(1);
            DubinsWrapper wrapper(s, current, 1); // turning radius doesn't matter because it will be straight
            stats.Plan.append(wrapper);
            // TODO! -- if we finish a line we should continue along the same heading for another step
            localRibbonManager.coverBetween(s.x(), s.y(), current.x(), current.y(), true);
        } else {
            current.time() += 1;
        }
//        auto currentNRibbons = localRibbonManager.get().size();
//        auto aheadState = current.push(1);
//        localRibbonManager.coverBetween(current.x(), current.y(), aheadState.x(), aheadState.y());
//        auto newNRibbons = localRibbonManager.get().size();
//        if (newNRibbons < currentNRibbons) {
//            // force next direction to be the same we just did
//            nextDir = direction;
//        }
    }
    return stats;
}
