cmake_minimum_required(VERSION 3.8)
project(lupin_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Interfaces will be added as the project develops, e.g.:
#   "msg/FlowerDetection.msg"
#   "srv/SetMode.srv"
#   "action/NavigateToTag.action"
set(msg_files
)
set(srv_files
)
set(action_files
)

if(msg_files OR srv_files OR action_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
