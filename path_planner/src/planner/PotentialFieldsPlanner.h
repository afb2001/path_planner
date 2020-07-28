#ifndef SRC_POTENTIALFIELDSPLANNER_H
#define SRC_POTENTIALFIELDSPLANNER_H


#include "Planner.h"

class PotentialFieldsPlanner : public Planner {
public:
    ~PotentialFieldsPlanner() override = default;

    Stats plan(const RibbonManager& ribbonManager, const State& start, PlannerConfig config,
               const DubinsPlan& previousPlan, double timeRemaining) override;

private:
    struct Force {
        Force() = default;
        Force(double magnitude, double direction) {
            X = magnitude * cos(direction);
            Y = magnitude * sin(direction);
        }
        double X, Y;
        Force operator+(const Force& other) const {
            Force result{};
            result.X = X + other.X;
            result.Y = Y + other.Y;
            return result;
        }
        Force operator-(const Force& other) const {
            Force result{};
            result.X = X - other.X;
            result.Y = Y - other.Y;
            return result;
        }

        double getDirection() const {
            // radians north of east
            return atan2(Y, X);
        }
    };

    static double getRibbonMagnitude(double distance)  {
        // avoid dividing by 0 with a max value
        if (distance <= 0.5) return 20;
        return 10 / distance;
    }

    static double getDynamicObstacleMagnitude(double distance, double width, double length) {
        // if we're super close just return a really high value
        if (distance <= 0) return 1000;
        // scale magnitude by obstacle area
        return exp(-distance / 10) * width * length / 100;
    }

    static double getStaticObstacleMagnitude(double distance) {
        if (distance > c_StaticObsIgnoreThreshold) return 0;
        return exp(-distance / 10);
    }

    static constexpr int c_LookaheadSteps = 10;

    static constexpr double c_StaticObsIgnoreThreshold = 7.5;
};


#endif //SRC_POTENTIALFIELDSPLANNER_H
