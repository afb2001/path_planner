#ifndef SRC_GRIDWORLDMAP_H
#define SRC_GRIDWORLDMAP_H

#include <vector>
#include "Map.h"

class GridWorldMap : public Map {
public:
    GridWorldMap(const std::string& path);

    ~GridWorldMap() override = default;

    double getUnblockedDistance(double x, double y) const override;

private:
    std::vector<std::vector<double>> m_Distances;
    int m_Resolution;
};


#endif //SRC_GRIDWORLDMAP_H
