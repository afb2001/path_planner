#ifndef SRC_EDGE_H
#define SRC_EDGE_H


#include <boost/exception/detail/shared_ptr.hpp>
#include "Vertex.h"
extern "C" {
#include "dubins.h"
}
#include "../common/Map.h"
#include "../common/DynamicObstacles.h"
#include "../common/Path.h"
#include "../common/Plan.h"

#define TIME_PENALTY 1

//int dubins_shortest_path(DubinsPath*, double*, double*, double);
//int dubins_path_length(DubinsPath*);

class Vertex;
class Edge {
public:
    DubinsPath dubinsPath;
    std::vector<State> plan;

    Edge(std::shared_ptr<Vertex> start);

    Edge(std::shared_ptr<Vertex> start, const State& end);

    ~Edge();

    std::shared_ptr<Vertex> setEnd(const State& state);

    double
    computeTrueCost(Map *map, DynamicObstacles *obstacles, const Path &toCover, double maxSpeed, double maxTurningRadius);

    double computeApproxCost(double maxSpeed, double maxTurningRadius);

    void smooth(Map* map, DynamicObstacles* obstacles, double maxSpeed, double maxTurningRadius);

    Plan getPlan(double maxSpeed);

    std::shared_ptr<Vertex> start();

    std::shared_ptr<Vertex> end();

private:
    std::shared_ptr<Vertex> m_Start;
    std::weak_ptr<Vertex> m_End;

    double m_ApproxCost = -1, m_TrueCost = -1;

    double netTime();
};


#endif //SRC_EDGE_H
