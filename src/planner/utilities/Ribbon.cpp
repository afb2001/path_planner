#include <sstream>
#include "Ribbon.h"

Ribbon::Ribbon(double startX, double startY, double endX, double endY)
    : m_StartX(startX), m_StartY(startY), m_EndX(endX), m_EndY(endY){}

Ribbon Ribbon::split(double x, double y) {
    // Project <x, y> onto the line spanned by this ribbon
    auto projected = getProjection(x, y);
    if (!contains(x, y, projected)) return Ribbon::empty();
    // Split the ribbon at the projected coordinates
    Ribbon r(m_StartX, m_StartY, projected.first, projected.second);
    m_StartX = projected.first; m_StartY = projected.second;
    return r;
}

Ribbon Ribbon::empty() {
    return Ribbon(0, 0, 0, 0);
}

bool Ribbon::covered() const {
    return squaredLength() < c_MinSquaredLength;
}

double Ribbon::length() const {
    return sqrt(squaredLength());
}

std::pair<double, double> Ribbon::start() const {
    return std::make_pair(m_StartX, m_StartY);
}

std::pair<double, double> Ribbon::end() const {
    return std::make_pair(m_EndX, m_EndY);
}

bool Ribbon::contains(double x, double y, const std::pair<double, double>& projected) const {
    if (((projected.first - m_StartX < -c_Tolerance && projected.first - m_EndX < -c_Tolerance) ||
         (projected.first - m_StartX > c_Tolerance && projected.first - m_EndX > c_Tolerance)) ||
        ((projected.second - m_StartY < -c_Tolerance && projected.second - m_EndY < -c_Tolerance) ||
            (projected.second - m_StartY > c_Tolerance && projected.second - m_EndY > c_Tolerance))) return false;
    auto d = distance(x, y);
    return d < c_RibbonWidth;
}

std::string Ribbon::toString() const {
    std::stringstream stream;
    stream << "(" << m_StartX << ", " << m_StartY << ") -> (" << m_EndX << ", " << m_EndY << ")"
        << " with length " << length();
    return stream.str();
}

double Ribbon::minLength() {
    return c_MinLength;
}

State Ribbon::startAsState() const {
    State s(m_StartX, m_StartY, 0, 0, 0);
    s.setHeadingTowards(m_EndX, m_EndY);
    return s;
}

State Ribbon::endAsState() const {
    State s(m_EndX, m_EndY, 0, 0, 0);
    s.setHeadingTowards(m_StartX, m_StartY);
    return s;
}

std::pair<double, double> Ribbon::getProjection(double x, double y) const {
    auto squaredL = squaredLength();
    auto dot = (x - m_StartX) * (m_EndX - m_StartX) + (y - m_StartY) * (m_EndY - m_StartY);
    auto projectedX = (m_EndX - m_StartX) * dot / squaredL;
    auto projectedY = (m_EndY - m_StartY) * dot / squaredL;
    return std::make_pair(projectedX + m_StartX, projectedY + m_StartY);
}

State Ribbon::getProjectionAsState(double x, double y) const {
    auto squaredL = squaredLength();
    auto dot = (x - m_StartX) * (m_EndX - m_StartX) + (y - m_StartY) * (m_EndY - m_StartY);
    auto projectedX = (m_EndX - m_StartX) * dot / squaredL;
    auto projectedY = (m_EndY - m_StartY) * dot / squaredL;
    State s(projectedX + m_StartX, projectedY + m_StartY, 0, 0, 0);
    s.setHeadingTowards(m_EndX, m_EndY);
    return s;
}



