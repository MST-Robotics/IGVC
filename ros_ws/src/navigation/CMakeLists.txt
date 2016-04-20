cmake_minimum_required(VERSION 2.8.3)
project(navigation)

set (NAVSOURCES src/euclidiandistancehueristic.cpp
src/PathFinderTutorials.cpp)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
include/navigation/
)


 add_executable(navigation_node ${NAVSOURCES})


 add_dependencies(navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigation_node
   ${catkin_LIBRARIES}
 )

