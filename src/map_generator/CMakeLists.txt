cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(

  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_library(map_generator 
  src/map_generator.cpp 
  src/Astar_searcher.cpp
  src/JPS_utils.cpp
  src/JPS_searcher.cpp
)

# add_executable(map_generator
#   src/map_generator.cpp 
#   src/Astar_searcher.cpp
#   src/JPS_utils.cpp
#   src/JPS_searcher.cpp
# )


target_link_libraries(map_generator
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

add_dependencies(map_generator
  ${catkin_EXPORTED_TARGETS}
)



