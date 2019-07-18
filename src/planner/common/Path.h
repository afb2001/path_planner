#ifndef SRC_PATH_H
#define SRC_PATH_H

#include <vector>
#include <utility>
#include "path_planner/State.h"

class Path {
public:
    Path();

    explicit Path(std::vector<std::pair<double, double>> points);

    void add(double x, double y);

    void add(const std::pair<double, double>& point);

    void add(const std::vector<std::pair<double, double>>& points);

    void clear();

    const std::vector<std::pair<double, double>>& get() const;

    std::vector<std::pair<double, double>> removeNewlyCovered(double x, double y);
    std::vector<std::pair<double, double>> removeNewlyCovered(const std::pair<double, double>& point);

    bool remove(const std::pair<double, double> &point);

    void remove(const std::vector<std::pair<double, double>>& points);

    int size() const;

    double maxDistanceFrom(const State& state);

    static bool covers(std::pair<double, double> p1, std::pair<double, double> p2);
    static bool covers(std::pair<double, double> p, double x, double y);
    static bool covers(double x1, double y1, double x2, double y2);

private:
    std::vector<std::pair<double, double>> m_Points;
};



#endif //SRC_PATH_H
