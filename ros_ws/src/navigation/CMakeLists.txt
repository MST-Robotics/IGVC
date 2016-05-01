cmake_minimum_required(VERSION 2.8.3)
project(navigation)

set (NAVSOURCES src/euclidiandistancehueristic.cpp
src/navigation_node.cpp src/navigation.cpp src/minheap.cpp include/navigation/astar.hpp include/navigation/minheap.hpp include/navigation/updatepriorityqueue.hpp include/navigation/astar.h 
include/navigation/euclidiandistancehueristic.h include/navigation/minheap.h include/navigation/navigation.h include/navigation/typewkey.h
include/navigation/updatepriorityqueue.h)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs nav_msgs
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

