#include <fstream>
#include <sstream>
#include <queue>
#include <cmath>
#include <cfloat>
#include "GridWorldMap.h"

double GridWorldMap::getUnblockedDistance(double x, double y) const {
    return m_Distances.at(y / m_Resolution).at(x / m_Resolution);
}

GridWorldMap::GridWorldMap(const std::string& path) {
    // read file into collection of strings
    std::ifstream infile(path);
    std::string line;
    std::vector<std::string> lines;
    std::getline(infile, line);
    std::istringstream s(line);
    s >> m_Resolution;
    int cols = -1, rows = 0;
    while (std::getline(infile, line)) {
        if (cols == -1) cols = line.length();
        else if (line.length() < cols) cols = line.length();
        rows++;
        lines.push_back(line);
    }

    class BrushFireCell {
    public:
        BrushFireCell(int x, int y, double distanceToBlocked) : x(x), y(y), DistanceToBlocked(distanceToBlocked) {}
        double DistanceToBlocked;
        int x, y;
        void pushNeighbors(std::queue<BrushFireCell>& queue, int xMax, int yMax, int resolution) {
            for (int i = x - 1; i <= x + 1; i++) {
                for (int j = y - 1; j <= y + 1; j++){
                    if (i == x && j == y) continue;
                    auto d = sqrt((i - x)*(i - x)*resolution*resolution + (j - y)*(j - y)*resolution*resolution);
                    if (DistanceToBlocked == -1) d = 1;
                    BrushFireCell cell(i, j, DistanceToBlocked + d);
                    if (cell.valid(xMax, yMax)) queue.push(cell);
                }
            }
        }
        bool valid(int xMax, int yMax) const {
            return 0 <= x && 0 <= y && x < xMax && y < yMax;
        }
    };
    std::queue<BrushFireCell> brushFireQueue;
    m_Distances = std::vector<std::vector<double>>(rows, std::vector<double>(cols, DBL_MAX));

    // do brushfire over the strings into m_Distances
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (lines[y][x] == '#') {
                brushFireQueue.push(BrushFireCell(x, y, -1));
            }
        }
    }

    while (!brushFireQueue.empty()) {
        auto& cell = brushFireQueue.front();
        if (m_Distances[cell.y][cell.x] > cell.DistanceToBlocked) { // cell has not been set yet (or found shorter distance?)
            m_Distances[cell.y][cell.x] = cell.DistanceToBlocked;
            cell.pushNeighbors(brushFireQueue, cols, rows, m_Resolution);
        }
        brushFireQueue.pop();
    }
}
