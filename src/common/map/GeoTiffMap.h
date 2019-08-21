#ifndef SRC_GEOTIFFMAP_H
#define SRC_GEOTIFFMAP_H

#include <gdal_priv.h>
#include <string>
#include "Map.h"

class GeoTiffMap : public Map {
public:
    explicit GeoTiffMap(std::string path);

    ~GeoTiffMap() override = default;

    float getDepth(double x, double y);

    double getUnblockedDistance(double x, double y) const override;

private:
    GDALDataset* m_Dataset;
    std::vector<std::vector<float>> m_Data;
    std::vector<double> m_InverseGeoTransform;
};


#endif //SRC_GEOTIFFMAP_H
