# Helmoro joint command message

std_msgs/Header header              # Header stamp should be acquisition time of measure.
int8 mode
float64[4] command                  # Commanded ; Position[rad]; Velocity[rad/s]; Effort[Nm]
