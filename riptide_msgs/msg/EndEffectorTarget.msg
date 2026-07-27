# Desired end-effector state for the stabilization controllers.
# header.frame_id names the reference frame the target is expressed in
# (typically "world"), so the target is meaningful regardless of base motion.
std_msgs/Header header

geometry_msgs/Pose  pose     # desired EE pose
geometry_msgs/Twist twist    # desired EE velocity (feed-forward)
geometry_msgs/Accel accel    # desired EE acceleration (feed-forward)
