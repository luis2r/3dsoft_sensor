Header header

################################################
# GPS status information

septentrio_asterx2_status status

################################################
# GPS coordinates information

float64 latitude
float64 longitude
float64 altitude

################################################
# GPS covariance information

float64[9] cov_pose_geod
float64[9] cov_orientation
float64[9] cov_velocity

################################################
# orientation vector contains heading, roll, pitch: INS attitude in radians
# orientation.x == roll
# orientation.y == pitch
# orientation.z == heading/yaw

geometry_msgs/Vector3 orientation

################################################
# orientation_flag vector means which measurement is valid
# orientation_flag.x == True,  means orientation.x is publishing
# orientation_flag.y == True,  means orientation.y is publishing
# orientation_flag.z == False, means orientation.z is NOT publishing

bool_vector3 orientation_flag

################################################
# velocity  vector on ECEF coordinate system

geometry_msgs/Vector3 velocity

################################################
#  GPS global time 

float64 gps_time

################################################
# cog: Course over ground: this is defined as the angle of the vehicle
#      with respect to the local level North, ranging from 0 to 2PI,
#      and increasing towards east. Set to the do-not-use value when
#      the speed is lower than 0.1m/s. COG is in radians.

float32 cog
