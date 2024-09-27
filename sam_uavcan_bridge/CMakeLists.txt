cmake_minimum_required(VERSION 3.8)
project(sam_uavcan_bridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sam_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(uavcan_ros_msgs REQUIRED)

# Set Canard base directory
set(CANARD_BASE ${PROJECT_SOURCE_DIR}/libcanard)
set(DSDL_DIR ${PROJECT_SOURCE_DIR}/dsdl)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/ros_to_uavcan/
  ${PROJECT_SOURCE_DIR}/include/uavcan_to_ros/
  ${PROJECT_SOURCE_DIR}/include/uavcan_ros_bridge
  ${PROJECT_SOURCE_DIR}/dsdl
  ${CANARD_BASE}
  ${CANARD_BASE}/drivers/socketcan
  ${DSDL_DIR}/dsdl_generated/include/
  ${DSDL_DIR}/dsdl_generated/src/
)

# Enable C++ API
add_definitions(-DDRONECAN_CXX_WRAPPERS)

# Add Canard library sources
set(CANARD_SOURCES
  ${CANARD_BASE}/canard.c
  ${CANARD_BASE}/drivers/socketcan/socketcan.c
)

# Add generated DSDL sources
set(DSDL_SOURCES
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.NodeStatus.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.GetNodeInfo_res.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.GetNodeInfo_req.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.equipment.esc.RawCommand.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.equipment.esc.Status.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.param.GetSet_req.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.param.GetSet_res.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
  ${DSDL_DIR}/dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
)


set(MESSAGE_CPP_FILES
    ${DSDL_DIR}/dsdl_generated/src/*c
)
set(CONVERT_FUNCTIONS
    ${PROJECT_SOURCE_DIR}/src/ros_to_uavcan/*cpp
    ${PROJECT_SOURCE_DIR}/src/uavcan_to_ros/*cpp)
    file(GLOB_RECURSE MESSAGE_CPP_FILES_FILES ${MESSAGE_CPP_FILES})
    file(GLOB_RECURSE CONVERT_FUNCTIONS_FILES ${CONVERT_FUNCTIONS})
# Create Canard library
add_library(canard_lib ${CANARD_SOURCES})
target_compile_options(canard_lib PRIVATE -Wall -Wextra -pedantic)

set(CONVERT_DEPENDS
std_msgs
rclcpp
std_srvs
sam_msgs
smarc_msgs
uavcan_ros_msgs
sensor_msgs
)


# UAVCAN to ROS messages library
add_library(uav_to_ros_msgs
  src/uavcan_to_ros/pressure.cpp
  src/uavcan_to_ros/battery_info.cpp
  src/uavcan_to_ros/panic.cpp
  src/uavcan_to_ros/actuator_status.cpp
  src/uavcan_to_ros/battery_state_basic.cpp
  src/uavcan_to_ros/circuit_status_stamped.cpp
  src/uavcan_to_ros/consumed_charge_array.cpp
  src/uavcan_to_ros/consumed_charge_feedback.cpp
  src/uavcan_to_ros/ctd_feedback.cpp
  src/uavcan_to_ros/dual_thruster_feedback.cpp
  src/uavcan_to_ros/leak.cpp
  src/uavcan_to_ros/sensor_pressure_stamped.cpp
  src/uavcan_to_ros/servo_feedback_double.cpp
  src/uavcan_to_ros/temperature.cpp
  src/uavcan_to_ros/thruster_feedback_id.cpp
  src/uavcan_to_ros/esc_status.cpp
  src/uavcan_to_ros/imu.cpp
  src/uavcan_to_ros/gps_fix.cpp
  src/uavcan_to_ros/magnetic_field.cpp
  src/uavcan_to_ros/sensor_pressure.cpp
)

ament_target_dependencies(uav_to_ros_msgs
  ${CONVERT_DEPENDS}
)

# ROS to UAVCAN messages library
add_library(ros_to_uav_msgs
  src/ros_to_uavcan/command.cpp
  src/ros_to_uavcan/percent_stamped.cpp
  src/ros_to_uavcan/ballast_angles.cpp
  src/ros_to_uavcan/dropweights.cpp
  src/ros_to_uavcan/dvl.cpp
  src/ros_to_uavcan/sss.cpp
  src/ros_to_uavcan/led.cpp
  src/ros_to_uavcan/thruster_rpm.cpp
  src/ros_to_uavcan/thruster_rpm_id.cpp
  src/ros_to_uavcan/thruster_rpms.cpp
  src/ros_to_uavcan/thruster_angles.cpp
  src/ros_to_uavcan/dual_thruster_rpm.cpp
  src/ros_to_uavcan/light_command.cpp
  src/ros_to_uavcan/panic.cpp
  src/ros_to_uavcan/array_command.cpp
)

ament_target_dependencies(ros_to_uav_msgs
  ${CONVERT_DEPENDS}
)

# Services library
add_library(ros_to_uavcan_services_library
  src/ros_to_uavcan/uavcan_node_info.cpp
  src/ros_to_uavcan/uavcan_update_battery.cpp
  src/ros_to_uavcan/uavcan_restart.cpp
  src/ros_to_uavcan/uavcan_transport_stats.cpp

)

ament_target_dependencies(ros_to_uavcan_services_library
  ${CONVERT_DEPENDS}
)

# add_custom_command(
#   OUTPUT ${DSDL_DIR}/dsdl_generated
#   COMMAND python3 ${DSDL_DIR}/dronecan_dsdlc/dronecan_dsdlc.py -O ${DSDL_DIR}/dsdl_generated ${DSDL_DIR}/DSDL/dronecan ${DSDL_DIR}/DSDL/uavcan ${DSDL_DIR}/DSDL/com ${DSDL_DIR}/DSDL/ardupilot ${DSDL_DIR}/smarc_uavcan_messages ${DSDL_DIR}/smarc_uavcan_services
#   DEPENDS ${DSDL_DIR}/DSDL ${DSDL_DIR}/dronecan_dsdlc
#   WORKING_DIRECTORY ${DSDL_DIR}
# )w
add_custom_target(generate_dsdl DEPENDS ${DSDL_DIR}/dsdl_generated)



# Create DSDL library
add_library(dsdl_lib ${DSDL_SOURCES})
target_compile_options(dsdl_lib PRIVATE -Wall -Wextra -pedantic)

# Create executable
add_executable(ros_to_uavcan_bridge src/ros_to_uavcan_bridge.cpp src/canard_interface.cpp src/time_utils.cpp ${CONVERT_FUNCTIONS_FILES} ${MESSAGE_CPP_FILES_FILES})
add_executable(ros_to_uavcan_services src/ros_to_uavcan_services.cpp src/canard_interface.cpp src/time_utils.cpp ${CONVERT_FUNCTIONS_FILES} ${MESSAGE_CPP_FILES_FILES})
add_executable(uavcan_ros_bridge src/uavcan_to_ros_bridge.cpp src/canard_interface.cpp src/time_utils.cpp  ${CONVERT_FUNCTIONS_FILES} ${MESSAGE_CPP_FILES_FILES})
# add_dependencies(ros_to_uavcan_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} )
add_dependencies(ros_to_uavcan_bridge generate_dsdl)
add_dependencies(ros_to_uavcan_services generate_dsdl)
add_dependencies(uavcan_ros_bridge generate_dsdl)
ament_target_dependencies(ros_to_uavcan_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  sam_msgs
  smarc_msgs
)
ament_target_dependencies(ros_to_uavcan_services
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  sam_msgs
  smarc_msgs
  uavcan_ros_msgs
)
ament_target_dependencies(uavcan_ros_bridge
rclcpp
geometry_msgs
sensor_msgs
std_msgs
std_srvs
sam_msgs
smarc_msgs
uavcan_ros_msgs
)
# Link libraries
target_link_libraries(ros_to_uavcan_bridge
  canard_lib
  dsdl_lib
  ros_to_uav_msgs

)
target_link_libraries(ros_to_uavcan_services
canard_lib
dsdl_lib
ros_to_uavcan_services_library
)

target_link_libraries(uavcan_ros_bridge
canard_lib
dsdl_lib
uav_to_ros_msgs)

target_compile_options(ros_to_uavcan_bridge PRIVATE  -Wno-unused-parameter
)
target_compile_options(uavcan_ros_bridge PRIVATE  -Wno-unused-parameter
)
target_compile_options(ros_to_uavcan_services PRIVATE  -Wno-unused-parameter
)

# Install targets
install(TARGETS
  ros_to_uavcan_bridge
  ros_to_uavcan_services
  uavcan_ros_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()