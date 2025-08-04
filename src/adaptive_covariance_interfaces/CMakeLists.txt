cmake_minimum_required(VERSION 3.8)
project(adaptive_covariance_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SynchronizedSensorData.msg"
  "msg/SensorSyncStatus.msg"
  "srv/GetSyncStatus.srv"
  DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    builtin_interfaces
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
