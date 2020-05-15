#ifndef SRC_GRIDWORLDMAP_H
#define SRC_GRIDWORLDMAP_H

#include <vector>
#include "Map.h"

/**
 * Represent a map loaded from a grid-world text file.
 */
class GridWorldMap : public Map {
public:
    /**
     * Load the map from a text file at the given path.
     * @param path
     */
    GridWorldMap(const std::string& path);

    ~GridWorldMap() override = default;

    double getUnblockedDistance(double x, double y) const override;

private:
    std::vector<std::vector<double>> m_Distances;
    int m_Resolution;
};


#endif //SRC_GRIDWORLDMAP_H
