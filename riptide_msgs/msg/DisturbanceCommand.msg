# External wrench to inject into the simulator (R4: heavy disturbances).
# The disturbance applier in riptide_mujoco maps `body` -> mjData.xfrc_applied.
# The same message is echoed on a ground-truth topic for evaluation.
std_msgs/Header header

string body                  # link/body the wrench is applied to (e.g. "auv_base_link")
geometry_msgs/Wrench wrench  # force (N) + torque (Nm), expressed in `header.frame_id`
