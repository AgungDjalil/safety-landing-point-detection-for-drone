cmake_minimum_required(VERSION 3.8)
project(gz_bridge_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- drone_kinematic node ------------------------------------------------
# Subscribes to /fmu/out/vehicle_odometry (PX4 -> ROS 2 via microXRCE-DDS) and
# broadcasts the dynamic drone TF tree:
#   map -> odom -> base_link   (odom -> base_link is dynamic, ~200 Hz)
# plus the static legs:
#   map -> odom                (identity)
#   base_link -> camera_link   (camera mount, SDF-exact)
add_executable(drone_kinematic src/drone_kinematic.cpp)
ament_target_dependencies(drone_kinematic
  rclcpp
  px4_msgs
  geometry_msgs
  tf2_ros
  builtin_interfaces
  tf2_geometry_msgs
)

install(TARGETS
  drone_kinematic
  DESTINATION lib/${PROJECT_NAME}
)

# ---- launch + config -----------------------------------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---- mavlink_cli: interactive MAVProxy wrapper for PX4 commander ---------
# `ros2 run gz_bridge_ros2 mavlink_cli` opens a MAVProxy shell to PX4 SITL
# over UDP 14550 (the GCS broadcast port). Interactive-only — do NOT launch
# this from ros2 launch (stdin is not reliably forwarded by launch).
install(
  PROGRAMS scripts/mavlink_cli.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME mavlink_cli
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # The `perception` mode table in launch/depth_bridge_launch.py. Its
  # conditions are substitutions evaluated at launch time, so a mistake there
  # produces a launch that looks healthy while starting nothing -- no error to
  # notice. Pinned here instead.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_depth_bridge_launch test/test_depth_bridge_launch.py)
endif()

ament_package()