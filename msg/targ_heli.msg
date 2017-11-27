#lib/goe
# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#
float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
float32 vel_n			# North velocity in NED earth-fixed frame, (metres/sec)
float32 vel_e			# East velocity in NED earth-fixed frame, (metres/sec)
float32 vel_d			# Down velocity in NED earth-fixed frame, (metres/sec)
float32 yaw 			# Euler yaw angle relative to NED earth-fixed frame, -PI..+PI, (radians)