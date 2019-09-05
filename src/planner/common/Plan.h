#ifndef SRC_PLAN_H
#define SRC_PLAN_H


#include <vector>
#include <path_planner/State.h>

#define TIME_HORIZON 30
#define TIME_MINIMUM 5
#define PLAN_TIME_DENSITY 0.5

class Plan {
public:
    void append(const State& s);

    void append(const Plan& plan);

    std::vector<State> get() const;

    const std::vector<State>& getRef() const;

    std::string toString() const;
private:
    std::vector<State> m_States;
};


#endif //SRC_PLAN_H
