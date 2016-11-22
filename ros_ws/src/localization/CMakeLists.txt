cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    message_generation
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(FILES rpy.msg)
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES localization
        CATKIN_DEPENDS roscpp tf message_generation std_msgs
        DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        ${catkin_INCLUDE_DIRS}
        include/localization/
)

add_executable(orientation_node src/localization/orientation_node.cpp include/localization/orientation.h
               src/localization/orientation.cpp)
add_dependencies(orientation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(orientation_node ${catkin_LIBRARIES})
