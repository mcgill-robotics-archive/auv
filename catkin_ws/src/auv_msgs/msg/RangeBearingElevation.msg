# Don't forget to set the sensor frame in the header
Header header

# The name of the object, which will be the name of the tf frame 
# published representing this object
string name

# Natural distance from sensor to target
float64 range
# The variance of ln(range)
float64 ln_range_variance

# Relative bearing: The angle between the projection of the vector
# from the robot to the target into the xy plane and the x-axis in
# radians e.g. if the target is along the y-axis, its bearing is pi/2
float64 bearing
float64 bearing_variance
# Elevation: the angle between the vectr from the robot to the 
# target and the xy-plane e.g. if the target is along the z-axis its
# elevation is pi/2. If elevation is not available, set the variance
# of elevation to be large.
float64 elevation
float64 elevation_variance



