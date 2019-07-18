#ifndef SRC_EDGE_H
#define SRC_EDGE_H


#include "Vertex.h"
#include <robust_dubins/RobustDubins.h>
#include "../common/Map.h"
#include "../common/DynamicObstaclesManager.h"
#include "../common/Path.h"
#include "../common/Plan.h"

extern "C" {
#include "dubins.h"
}

#define DUBINS_INCREMENT 0.1
#define TIME_PENALTY 1
#define COLLISION_PENALTY 600

//int dubins_shortest_path(DubinsPath*, double*, double*, double);
//int dubins_path_length(DubinsPath*);

class Vertex;

class Edge {
public:
    DubinsPath dubinsPath;

    Edge(std::shared_ptr<Vertex> start);

    Edge(std::shared_ptr<Vertex> start, const State& end);

    ~Edge();

    std::shared_ptr<Vertex> setEnd(const State& state);

    double computeTrueCost(Map *map, DynamicObstaclesManager *obstacles, double maxSpeed, double maxTurningRadius);

    double trueCost() const;

    double computeApproxCost(double maxSpeed, double maxTurningRadius);

    void smooth(Map* map, DynamicObstaclesManager* obstacles, double maxSpeed, double maxTurningRadius);

    Plan getPlan(double maxSpeed);

    std::shared_ptr<Vertex> start();

    std::shared_ptr<Vertex> end();

private:
    std::shared_ptr<Vertex> m_Start;
    std::weak_ptr<Vertex> m_End;

    double m_ApproxCost = -1, m_TrueCost = -1;
public:
    double approxCost() const;

private:

    double netTime();
};


#endif //SRC_EDGE_H
