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
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/SensorReading.msg"
  "msg/TagReading.msg"
  "msg/FlowerObservation.msg"
  "msg/AnomalyReport.msg"
  "msg/Observation.msg"
  "msg/MissionState.msg"
  "msg/TwinTagState.msg"
  "msg/TwinState.msg"
)
set(srv_files
  "srv/GetTagReading.srv"
  "srv/StartMission.srv"
  "srv/SetArmPreset.srv"
  "srv/GetField.srv"
)
set(action_files
)

if(msg_files OR srv_files OR action_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
