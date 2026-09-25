cmake_minimum_required(VERSION 3.8)
project(riptide_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Optional helper nodes (Python scripts run via `ros2 run` / the launch):
#   * mujoco_viewer.py  — passive MuJoCo viewer (needs the `mujoco` pip package)
#   * ee_target_gui.py  — Tkinter EE-pose teleop (needs python3-tk)
install(
  PROGRAMS
    scripts/mujoco_viewer.py
    scripts/ee_target_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
