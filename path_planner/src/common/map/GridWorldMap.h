#ifndef SRC_GRIDWORLDMAP_H
#define SRC_GRIDWORLDMAP_H

#include "Map.h"

#include <string>
#include <vector>

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

    bool isBlocked(double x, double y) const override;

    const double* extremes() const override;

    double resolution() const override;

private:
    std::vector<std::vector<bool>> m_Blocked;
    double m_Resolution;
    double m_Extremes[4];
};


#endif //SRC_GRIDWORLDMAP_H
