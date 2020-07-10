#include <cfloat>
#include "Map.h"

bool Map::isBlocked(double x, double y) const {
    return false;
}

const double* Map::extremes() const {
    return m_Extremes;
}

int Map::resolution() const {
    return 0;
}
