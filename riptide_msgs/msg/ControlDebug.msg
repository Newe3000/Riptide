# Introspection output from the active control law, for logging/evaluation.
std_msgs/Header header

string   control_law          # e.g. "pid", "impedance", "lqr", "mpc"
float64  solve_time_ms        # wall time spent in compute()

string[]  joint_names         # order of the vectors below
float64[] tau                 # commanded joint torques [Nm]
float64[] q                   # measured joint positions
float64[] dq                  # measured joint velocities

float64[] ee_pose_error       # 6-vector: [x y z rx ry rz] task-space error
