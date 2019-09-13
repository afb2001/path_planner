#ifndef SRC_GEOTIFFMAP_H
#define SRC_GEOTIFFMAP_H

#include <gdal_priv.h>
#include <string>
#include "Map.h"

class GeoTiffMap : public Map {
public:
    explicit GeoTiffMap(const std::string& path, double longitude, double originLatitude);

    ~GeoTiffMap() override = default;

//    float getDepth(double x, double y) const;

    double getUnblockedDistance(double x, double y) const override;

private:
//    GDALDataset* m_Dataset;
//    std::vector<std::vector<float>> m_Data;
    std::vector<std::vector<double>> m_Distances;
    std::vector<double> m_InverseGeoTransform;
    double m_XOrigin, m_YOrigin;
    static constexpr double c_MinimumDepth = 0;
};


#endif //SRC_GEOTIFFMAP_H
