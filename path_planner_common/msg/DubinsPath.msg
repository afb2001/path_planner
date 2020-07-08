# Message representing a Dubins path. This correlates directly with my DubinsWrapper class.
# starting configuration
# x and y in map coordinates (meters)
float64 initial_x
float64 initial_y
# yaw in radians counter-clockwise from the x axis
float64 initial_yaw

# segment lengths (see dubins.h)
float64 length0
float64 length1
float64 length2

# turning radius (meters)
float64 rho

# path type: [LSL = 0, LSR = 1, RSL = 2, RSR = 3, RLR = 4, LRL = 5]
int32 type

# additional information (see DubinsWrapper.h)
# speed in meters/second
float64 speed
# initial time (seconds)
float64 start_time
