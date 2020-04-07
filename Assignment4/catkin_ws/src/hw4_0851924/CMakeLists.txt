cmake_minimum_required(VERSION 2.8.3)
project(hw4_0851924)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw4_0851924
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_node src/hw4_0851924_node.cpp)
add_executable(hw4_node src/hw4_node.cpp)


# add_dependencies(hw4_node sensor_msgs_generate_messages_cpp)
target_link_libraries(hw4_node
    ${catkin_LIBRARIES}
  )