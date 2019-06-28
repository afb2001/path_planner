#include <algorithm>
#include "Path.h"

Path::Path() = default;

Path::Path(std::vector<std::pair<double, double>> points) {
    m_Points = std::move(points);
}

void Path::remove(const std::pair<double, double> &point) {
    m_Points.erase(std::remove_if(m_Points.begin(), m_Points.end(), [&] (std::pair<double, double> p) {
        return p == point;
    }), m_Points.end());
}

void Path::remove(const std::vector<std::pair<double, double>>& points) {
    m_Points.erase(std::remove_if(m_Points.begin(), m_Points.end(), [&](std::pair<double, double> p){
        return std::find(points.begin(), points.end(), p) != points.end();
    }), m_Points.end());
}

void Path::add(const std::pair<double, double>& point) {
    m_Points.push_back(point);
}

void Path::add(const std::vector<std::pair<double, double>>& points) {
    for (auto p : points) add(p);
}

void Path::clear() {
    m_Points.clear();
}

std::vector<std::pair<double, double>> Path::get() const {
    return m_Points;
}

void Path::add(double x, double y) {
    m_Points.emplace_back(x, y);
}

int Path::size() const {
    return m_Points.size();
}

double Path::maxDistanceFrom(const State &state) {
    double max = 0;
    for (auto p : m_Points) {
        auto d = state.distanceTo(p.first, p.second);
        if (max < d) max = d;
    }
    return max;
}
