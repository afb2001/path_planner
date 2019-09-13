#include <algorithm>
#include "Path.h"

Path::Path() = default;

Path::Path(std::vector<std::pair<double, double>> points) {
    m_Points = std::move(points);
}

bool Path::remove(const std::pair<double, double> &point) {
    auto l = size();
    m_Points.erase(std::remove_if(m_Points.begin(), m_Points.end(), [&] (std::pair<double, double> p) {
        return p == point;
    }), m_Points.end());
    return l != size();
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

const std::vector<std::pair<double, double>>& Path::get() const {
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

std::vector<std::pair<double, double>> Path::removeNewlyCovered(double x, double y) {
    std::vector<std::pair<double, double>> newlyCovered;
    m_Points.erase(std::remove_if(m_Points.begin(), m_Points.end(), [&] (std::pair<double, double> p) {
        if (covers(p, x, y)) {
            newlyCovered.push_back(p);
            return true;
        }
        return false;
    }), m_Points.end());
    return newlyCovered;
}

bool Path::covers(double x1, double y1, double x2, double y2) {
    return ((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) < c_CoverageThreshold * c_CoverageThreshold);
}

bool Path::covers(std::pair<double, double> p1, std::pair<double, double> p2) {
    return covers(p1.first, p1.second, p2.first, p2.second);
}

bool Path::covers(std::pair<double, double> p, double x, double y) {
    return covers(x, y, p.first, p.second);
}

std::vector<std::pair<double, double>> Path::removeNewlyCovered(const std::pair<double, double>& point) {
    return removeNewlyCovered(point.first, point.second);
}

bool Path::covers(double distance) {
    return distance < c_CoverageThreshold;
}

double Path::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

double Path::distance(std::pair<double, double> p, double x, double y) {
    return distance(p.first, p.second, x, y);
}
