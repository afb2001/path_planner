#ifndef SRC_PATH_H
#define SRC_PATH_H

#include <vector>
#include <utility>

class Path {
public:
    Path();

    explicit Path(std::vector<std::pair<double, double>> points);

    void add(double x, double y);

    void add(const std::pair<double, double>& point);

    void add(const std::vector<std::pair<double, double>>& points);

    void clear();

    std::vector<std::pair<double, double>> get() const;

    void remove(const std::pair<double, double> &point);

    void remove(const std::vector<std::pair<double, double>>& points);

private:
    std::vector<std::pair<double, double>> m_Points;
};



#endif //SRC_PATH_H
