# Hamster robot sensor data message
std_msgs/Header header

# Proximity sensors (4 sensors: front, back, left, right)
float32[] proximity_sensors

# Acceleration sensor data
geometry_msgs/Vector3 acceleration

# Floor sensors (2 sensors: left, right)
bool[] floor_sensors

# Light sensor
float32 light_intensity

# Temperature (if available)
float32 temperature