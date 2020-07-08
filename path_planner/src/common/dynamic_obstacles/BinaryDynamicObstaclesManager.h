#ifndef SRC_BINARYDYNAMICOBSTACLESMANAGER_H
#define SRC_BINARYDYNAMICOBSTACLESMANAGER_H


#include <vector>
#include <unordered_map>
#include "DynamicObstaclesManager.h"
#include "DynamicObstaclesManagerBase.h"

class BinaryDynamicObstaclesManager : public DynamicObstaclesManagerBase {
public:
    typedef std::shared_ptr<BinaryDynamicObstaclesManager> SharedPtr;

    struct Obstacle {
        double X, Y, Yaw, Speed, Time;
        double Width, Length;
        Obstacle(double x, double y, double heading, double speed, double time, double width, double length)
                : X(x), Y(y), Yaw(M_PI_2 - heading), Speed(speed), Time(time), Width(width), Length(length){}
        void project(double desiredTime) {
            double dt = desiredTime - Time;
            double dx = Speed * dt * cos(Yaw);
            double dy = Speed * dt * sin(Yaw);
            X += dx; Y += dy;
        }
    };

    ~BinaryDynamicObstaclesManager() override = default;

    void update(uint32_t  mmsi, double x, double y, double heading, double speed, double time, double width, double length);

    void forget(uint32_t mmsi);

    double collisionExists(double x, double y, double time, bool strict) const override;

    const std::unordered_map<uint32_t, Obstacle>& get() const;

private:

    std::unordered_map<uint32_t, Obstacle> m_Obstacles;
};


#endif //SRC_BINARYDYNAMICOBSTACLESMANAGER_H
