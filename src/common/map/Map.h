#ifndef SRC_MAP_H
#define SRC_MAP_H


#include <memory>

class Map {
public:
    virtual ~Map() = default;
    typedef std::shared_ptr<Map> SharedPtr;
    virtual double getUnblockedDistance(double x, double y) const;
};


#endif //SRC_MAP_H
