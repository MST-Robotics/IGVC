cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    sensor_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate message in the 'msg' folder
add_message_files(FILES Encoder.msg)
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES control
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs
    DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########
include_directories(
    ${catkin_INCLUDE_DIRS}
    include/control/
    ../common/
)

add_executable(remote_control_node src/control/remote_control_node.cpp src/control/remote_control.cpp ../common/config.cpp)
add_dependencies(remote_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_control_node 
    ${catkin_LIBRARIES}
)

add_executable(teleop_mux_node src/control/twist_mux_node.cpp src/control/twist_mux.cpp ../common/config.cpp)
add_dependencies(teleop_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_mux_node 
    ${catkin_LIBRARIES}
)

add_executable(motor_controller_node src/control/motor_controller_node.cpp src/control/motor_controller.cpp)
add_dependencies(motor_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller_node
    ${catkin_LIBRARIES}
)


