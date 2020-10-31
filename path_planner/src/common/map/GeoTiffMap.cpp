#include <sstream>
#include <iostream>
#include <ogr_spatialref.h>
#include <cfloat>
#include <queue>
#include "GeoTiffMap.h"
#include <gdal_priv.h>

GeoTiffMap::GeoTiffMap(const std::string& path, double originLongitude, double originLatitude) {
    GDALAllRegister();
    auto dataset = static_cast<GDALDataset*>(GDALOpen(path.c_str(), GDALAccess::GA_ReadOnly));
    if (!dataset) throw std::runtime_error("GeoTiffMap failed to load map file");
    std::cerr << "Loading GEOTiff map from " << path << "...\n";
    auto band = dataset->GetRasterBand(1);
    auto geoTransform = new double[6];
    auto err1 = dataset->GetGeoTransform(geoTransform);
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

//    std::cerr << "Done reading map. Starting distances calculations" << std::endl;

    // set up projection so we know the origin
    char* projection = const_cast<char*> (dataset->GetProjectionRef());
    if (projection[0] == 0) projection = const_cast<char*>(dataset->GetGCPProjection());

    OGRSpatialReference projected, wgs84;
    projected.importFromWkt(&projection);
    wgs84.SetWellKnownGeogCS("WGS84");

    auto projectTransformation = OGRCreateCoordinateTransformation(&wgs84, &projected);

    m_XOrigin = originLongitude, m_YOrigin = originLatitude;
    projectTransformation->Transform(1, &m_XOrigin, &m_YOrigin);

//    // little local class for BrushFire algorithm
//    class BrushFireCell {
//    public:
//        BrushFireCell(int x, int y, double distanceToBlocked) : x(x), y(y), DistanceToBlocked(distanceToBlocked) {}
//        double DistanceToBlocked;
//        int x, y;
//        void pushNeighbors(std::queue<BrushFireCell>& queue, int xMax, int yMax, const double geoTransform[6]) {
//            for (int i = x - 1; i <= x + 1; i++) {
//                for (int j = y - 1; j <= y + 1; j++){
//                    if (i == x && j == y) continue;
//                    // This should fix the fact that the distance is in pixels...
//                    auto d = sqrt(((i - x)*geoTransform[1] + (j - y)*geoTransform[2])*((i - x)*geoTransform[1] + (j - y)*geoTransform[2]) +
//                                     ((i - x)*geoTransform[4] + (j - y)*geoTransform[5])*((i - x)*geoTransform[4] + (j - y)*geoTransform[5]));
//                    if (DistanceToBlocked == -1) d = 1;
//                    BrushFireCell cell(i, j, DistanceToBlocked + d);
//                    if (cell.valid(xMax, yMax)) queue.push(cell);
//                }
//            }
//        }
//        bool valid(int xMax, int yMax) const {
//            return 0 < x && 0 < y && x < xMax && y < yMax;
//        }
//    };
//    std::queue<BrushFireCell> brushFireQueue;
//    m_Distances = std::vector<std::vector<double>>(rasterRows, std::vector<double>(rasterCols, DBL_MAX));
//
    int blockedCount = 0;
    for (int y = 0; y < rasterRows; y++) {
        for (int x = 0; x < rasterCols; x++) {
            if (m_Data[y][x] < c_MinimumDepth) {
//                brushFireQueue.push(BrushFireCell(x, y, -1));
                blockedCount++;
//                std::cerr << "#";
            }
//            else {
//                std::cerr << "_";
//            }
        }
//        std::cerr << std::endl;
    }

    // prints out the map (upside down)
//    for (int y = 0; y < rows; y++) {
//        for (int x = 0; x < cols; x++) {
//            std::cerr << (m_Blocked[y][x]? "#" : "_");
//        }
//        std::cerr << std::endl;
//    }

    std::cerr << blockedCount << " out of " << rasterCols * rasterRows << " cells blocked" << std::endl;
//
//    while (!brushFireQueue.empty()) {
//        auto& cell = brushFireQueue.front();
//        if (m_Distances[cell.y][cell.x] > cell.DistanceToBlocked) { // cell has not been set yet (or found shorter distance?)
//            m_Distances[cell.y][cell.x] = cell.DistanceToBlocked;
//            cell.pushNeighbors(brushFireQueue, rasterCols, rasterRows, geoTransform);
//        }
//        brushFireQueue.pop();
//    }

    std::cerr << "Done loading map from " << path << std::endl;

    delete[] geoTransform;
}

float GeoTiffMap::getDepth(double x, double y) const {
    auto xi = m_InverseGeoTransform[0] + x * m_InverseGeoTransform[1] + y * m_InverseGeoTransform[2];
    auto yi = m_InverseGeoTransform[3] + x * m_InverseGeoTransform[4] + y * m_InverseGeoTransform[5];
    // bad: using exceptions for control flow
    try {
        return m_Data.at((int) yi).at((int) xi);
    } catch (std::out_of_range&) {
        return 0;
    }

}

bool GeoTiffMap::isBlocked(double x, double y) const {
    return getDepth(x, y) <= c_MinimumDepth;
}
