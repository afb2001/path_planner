#ifndef OBJECTPAR_H
#define OBJECTPAR_H

#include "string"
#include "iostream"
#include "cmath"


/**
 * This class represents a state of the vehicle in five dimensions: x, y, heading, speed, time.
 * Units: meters, radians east of north, m/s, seconds
 */
class State
{

  public:
    double x() const { return m_Pose[0]; }
    double& x() { return m_Pose[0]; }
    void setX(double x) { m_Pose[0] = x; }

    double y() const { return m_Pose[1]; }
    double& y() {return m_Pose[1]; }
    void setY(double y) { m_Pose[1] = y; }

    double heading() const { return m_Pose[2]; }
    double& heading() { return m_Pose[2]; }
    void setHeading(double heading) { m_Pose[2] = heading; }

    /**
     * Heading north of east. Val called it "yaw".
     * @return
     */
    double yaw() const {
        double h = M_PI_2 - heading();
        if (h < 0) h += 2 * M_PI;
        return h;
    }

    /**
     * Set the heading with a yaw value.
     * @param yaw1
     * @return
     */
    double yaw(double yaw1) {
        heading() = M_PI_2 - yaw1;
        if (heading() < 0) heading() += twoPi;
    }

    double speed() const { return m_Pose[3]; }
    double& speed() { return m_Pose[3]; }
    void setSpeed(double speed) { m_Pose[3] = speed; }

    double time() const { return m_Time; }
    double& time() { return m_Time; }
    void setTime(double time) { m_Time = time; }

    double* pose() { return m_Pose; }
    const double* pose() const { return m_Pose; }

    /**
     * Construct a State.
     * @param x
     * @param y
     * @param heading
     * @param speed
     * @param t
     */
    State(double x, double y, double heading, double speed, double t);

    State() = default;

    /**
     * Push this state forward for some amount of time (distance moved depends on speed).
     * @param timeInterval
     * @return the resultant state
     */
    State push(double timeInterval) const;

    /**
     * Pushes a state forward some distance. Doesn't affect time.
     * @param distance
     */
    void move(double distance);

    std::string toString() const;

    std::string toStringRad() const;

    /**
     * Get the direction (heading) towards another state.
     * @param other
     * @return the heading
     */
    double headingTo(const State& other) const;

    /**
     * Get the direction (heading) towards a point.
     * @param p
     * @return the heading
     */
    double headingTo(const std::pair<double, double> p) const;

    /**
     * Get the direction (heading) towards a point.
     * @param x1
     * @param y1
     * @return the heading
     */
    double headingTo(double x1, double y1) const;

    /**
     * Sets this state's heading to point towards the other state.
     * @param other
     */
    void setHeadingTowards(const State& other);

    /**
     * Sets this state's heading towards the point.
     * @param x1
     * @param y1
     */
    void setHeadingTowards(double x1, double y1);

    /**
     * Get the time difference between states.
     * @param other
     * @return the time until other
     */
    double timeUntil(const State& other) const;

    inline bool operator==(const State& rhs) const;

    /**
     * Determine whether two states share the same pose (ignores speed).
     * @param rhs
     * @return
     */
    bool isCoLocated(const State& rhs) const;

    /**
     * Get the Euclidean distance to the other state.
     * @param other
     * @return
     */
    double distanceTo(const State& other) const;

    /**
     * Get the Euclidean distance to the point.
     * @param x1
     * @param y1
     * @return
     */
    double distanceTo(double x1, double y1) const;

    /**
     * Interpolate (or extrapolate) between this state and other to the desired time.
     * @param other
     * @param desiredTime
     * @return
     */
    State interpolate(const State& other, double desiredTime) const;

    double headingDifference(const State& other) const;

    double headingDifference(double otherHeading) const;

private:
    double m_Pose [4] = {0, 0, 0, 0};
    double m_Time = -1;

    static constexpr double twoPi = 2 * M_PI;
};

#endif