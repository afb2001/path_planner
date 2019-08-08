#ifndef SRC_PLAN_H
#define SRC_PLAN_H


#include <vector>
#include <path_planner/State.h>

class Plan {
public:
    void append(const State& s);

    void append(const Plan& plan);

    std::vector<State> get() const;

    const std::vector<State>& getRef() const;

    std::string toString() const;

    static const double timeHorizon() { return c_TimeHorizon; }

    static const double timeMinimum() { return c_TimeMinimum; }

    static const double planTimeDensity() { return c_PlanTimeDensity; }
private:
    std::vector<State> m_States;

    static constexpr double c_TimeHorizon = 30;
    static constexpr double c_TimeMinimum = 5;
    static constexpr double c_PlanTimeDensity = 0.5;
};


#endif //SRC_PLAN_H
