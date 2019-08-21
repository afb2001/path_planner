#include <sstream>
#include <iostream>
#include "GeoTiffMap.h"

GeoTiffMap::GeoTiffMap(const std::string& path) {
    GDALAllRegister();
    m_Dataset = static_cast<GDALDataset*>(GDALOpen(path.c_str(), GDALAccess::GA_ReadOnly));
    if (!m_Dataset) throw std::runtime_error("GeoTiffMap failed to load map file");
    std::cerr << "Loading GEOTiff map from " << path << " ...\n";
    auto band = m_Dataset->GetRasterBand(1);
    auto geoTransform = new double[6];
    auto err1 = m_Dataset->GetGeoTransform(geoTransform);
    if (err1 == CPLErr::CE_Failure) {
        throw std::runtime_error("GeoTiffMap failed to find geo transform");
    }
    m_InverseGeoTransform = std::vector<double>(6);
    auto err2 = GDALInvGeoTransform(geoTransform, m_InverseGeoTransform.data());
    if (!err2) { // this is correct it's supposed to be a bool not a return code
        throw std::runtime_error("GeoTiffMap failed to invert geo transform");
    }
    int rasterCols = band->GetXSize(), rasterRows = band->GetYSize();
    m_Data = std::vector<std::vector<float>>(rasterRows);
    auto line = static_cast<float*>(CPLMalloc(sizeof(float) * rasterCols));
    // Read a row at a time. Efficiency shouldn't matter here and I'm more comfortable programming it this way
    for (int i = 0; i < rasterRows; i++) {
        auto err3 = band->RasterIO(GF_Read, 0, i, rasterCols, 1, line, rasterCols, 1, GDT_Float32, 0, 0);
        if (err3 != CE_None) {
            std::ostringstream stringStream;
            stringStream << "GeoTiffMap failed to access data at row " << i << "of band 1";
            throw std::runtime_error(stringStream.str());
        }
        for (int j = 0; j < rasterCols; j++) {
            m_Data[i].push_back(line[j]);
        }
    }
    CPLFree(line);
    delete[] geoTransform;
    std::cerr << "Done loading map from " << path << std::endl;
}

float GeoTiffMap::getDepth(double x, double y) {
    auto xi = m_InverseGeoTransform[0] + x * m_InverseGeoTransform[1] + y * m_InverseGeoTransform[2];
    auto yi = m_InverseGeoTransform[3] + x * m_InverseGeoTransform[4] + y * m_InverseGeoTransform[5];
    try {
        return m_Data.at((int)yi).at((int)xi);
    } catch(std::out_of_range&) {
        return 0;
    }

}

double GeoTiffMap::getUnblockedDistance(double x, double y) const {
    return Map::getUnblockedDistance(x, y); // TODO!
}
