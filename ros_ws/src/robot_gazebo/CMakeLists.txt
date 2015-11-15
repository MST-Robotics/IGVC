cmake_minimum_required(VERSION 2.8.3)
project(robot_gazebo)

find_package(catkin REQUIRED
    hector_gazebo_plugins
    gazebo_ros
    gazebo_plugins
    xacro
    tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    LIBRARIES robot_gazebo
    CATKIN_DEPENDS robot_description
    DEPENDS system_lib
)

