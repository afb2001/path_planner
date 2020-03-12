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
    State(double x, double y, double heading, double speed, double t)
    {
        setX(x);
        setY(y);
        setHeading(heading);
        setSpeed(speed);
        setTime(t);
    }

    State() = default;

    /**
     * Push this state forward for some amount of time (distance moved depends on speed).
     * @param timeInterval
     * @return the resultant state
     */
    State push(double timeInterval) const {
        State s;
        double displacement = timeInterval * speed();
        s.x() = x() + sin(heading()) * displacement;
        s.y() = y() + cos(heading()) * displacement;
        s.heading() = heading();
        s.speed() = speed();
        s.time() = time() + timeInterval;
        return s;
    }

    /**
     * Pushes a state forward some distance. Doesn't affect time.
     * @param distance
     */
    void move(double distance) {
        x() += cos(yaw()) * distance;
        y() += sin(yaw()) * distance;
    }

    std::string toString() const
    {
        return std::to_string(x()) + " " +
               std::to_string(y()) + " " +
               std::to_string(heading()*180/M_PI) + " " +
               std::to_string(speed()) + " " +
               std::to_string(time());
    }

    std::string toStringRad() const
    {
        return std::to_string(x()) + " " +
               std::to_string(y()) + " " +
               std::to_string(heading()) + " " +
               std::to_string(speed()) + " " +
               std::to_string(time());
    }

    /**
     * Get the direction (heading) towards another state.
     * @param other
     * @return the heading
     */
    double headingTo(const State& other) const {
        return headingTo(other.x(), other.y());
    }

    /**
     * Get the direction (heading) towards a point.
     * @param p
     * @return the heading
     */
    double headingTo(const std::pair<double, double> p) const {
        return headingTo(p.first, p.second);
    }

    /**
     * Get the direction (heading) towards a point.
     * @param x1
     * @param y1
     * @return the heading
     */
    double headingTo(double x1, double y1) const {
        double dx = x1 - this->x();
        double dy = y1 - this->y();
        double h = M_PI_2 - atan2(dy, dx); // TODO! -- is this correct?
        if (h < 0) h += 2 * M_PI;
        return h;
    }

    /**
     * Sets this state's heading to point towards the other state.
     * @param other
     */
    void setHeadingTowards(const State& other) {
        heading() = headingTo(other);
        if (heading() < 0) heading() += 2 * M_PI;
    }

    /**
     * Sets this state's heading towards the point.
     * @param x1
     * @param y1
     */
    void setHeadingTowards(double x1, double y1) {
        heading() = headingTo(x1, y1);
        if (heading() < 0) heading() += 2 * M_PI;
    }

    /**
     * Get the time difference between states.
     * @param other
     * @return the time until other
     */
    double timeUntil(const State& other) const {
        return other.time() - time();
    }

    inline bool operator==(const State& rhs) const {
        return x() == rhs.x() &&
                y() == rhs.y() &&
                heading() == rhs.heading() &&
                speed() == rhs.speed() &&
                time() == rhs.time();
    }

    /**
     * Determine whether two states share the same pose (ignores speed).
     * @param rhs
     * @return
     */
    bool isCoLocated(const State& rhs) const {
        return x() == rhs.x() &&
               y() == rhs.y() &&
               heading() == rhs.heading();
    }

    /**
     * Get the Euclidean distance to the other state.
     * @param other
     * @return
     */
    double distanceTo(const State& other) const {
        return distanceTo(other.x(), other.y());
    }

    /**
     * Get the Euclidean distance to the point.
     * @param x1
     * @param y1
     * @return
     */
    double distanceTo(double x1, double y1) const {
        return sqrt((this->x() - x1)*(this->x() - x1) + (this->y() - y1)*(this->y() - y1));
    }

    /**
     * Interpolate (or extrapolate) between this state and other to the desired time.
     * @param other
     * @param desiredTime
     * @return
     */
    State interpolate(const State& other, double desiredTime) const {
//        std::cerr << "Interpolating between " << other.toString() << " and " << toString() << std::endl;
        auto dt = other.time() - time();
        auto dx = (other.x() - x()) / dt;
        auto dy = (other.y() - y()) / dt;
        // assume the headings have changed the closer way, not all the way around the other way

        auto dh = headingDifference(other) / dt;
        auto ds = (other.speed() - speed()) / dt;
        dt = desiredTime - time();
        State s = *this;
        s.x() += dx * dt;
        s.y() += dy * dt;
        s.heading() = this->heading() + (dh * dt);
        if (s.heading() >= twoPi) s.heading() -= twoPi;
        s.speed() += ds * dt;
        s.time() = desiredTime;
        return s;
    }

    double headingDifference(const State& other) const {
        return headingDifference(other.heading());
    }

    double headingDifference(double otherHeading) const {
        return (fmod(fmod((otherHeading - heading()), twoPi) + 3 * M_PI, twoPi) - M_PI);
    }

private:
    double m_Pose [4] = {0, 0, 0, 0};
    double m_Time = -1;

    static constexpr double twoPi = 2 * M_PI;
};

#endif