cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES control
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs
    DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
    ${catkin_INCLUDE_DIRS}
    include/control/
    ../common/
)

add_executable(control_node src/remote_control_node.cpp src/remote_control.cpp)
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node
    ${catkin_LIBRARIES}
)
