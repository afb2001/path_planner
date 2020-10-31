#ifndef SRC_GEOTIFFMAP_H
#define SRC_GEOTIFFMAP_H

#include "Map.h"

#include <vector>
#include <string>

/**
 * Represent a map loaded from a GeoTiff.
 */
class GeoTiffMap : public Map {
public:
    /**
     * Loads a map from the given path. Currently converts it into a safety-distance grid but should skip that part and
     * just make a bitmap of occupancy.
     * @param path path to the map file.
     * @param longitude origin longitude
     * @param originLatitude origin latitude
     */
    explicit GeoTiffMap(const std::string& path, double longitude, double originLatitude);

    ~GeoTiffMap() override = default;

    float getDepth(double x, double y) const;

    bool isBlocked(double x, double y) const override;

    // TODO! -- to be useful in PF planner, override resolution function to allow querying at the right intervals

private:
//    GDALDataset* m_Dataset;
    std::vector<std::vector<float>> m_Data;
    std::vector<std::vector<double>> m_Distances;
    std::vector<double> m_InverseGeoTransform;
    double m_XOrigin, m_YOrigin;
    static constexpr double c_MinimumDepth = 0;
};


#endif //SRC_GEOTIFFMAP_H
