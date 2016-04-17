cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vision
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/vision/
)

add_executable(vision_node src/vision_node.cpp src/vision.cpp)
add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
