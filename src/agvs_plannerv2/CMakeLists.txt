cmake_minimum_required(VERSION 3.0.2)
project(agvs_plannerv2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  map_generator
  roscpp
  rospy
  std_msgs
  user_pkg
  message_generation
)

set(JSONCPP_INCLUDE_DIRS "/usr/include/jsoncpp")
set(JSONCPP_LIBRARIES "jsoncpp")

if (NOT EXISTS "${JSONCPP_INCLUDE_DIRS}/json/json.h")
  message(FATAL_ERROR "jsoncpp include directory not found: ${JSONCPP_INCLUDE_DIRS}")
endif()

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# set(OMPL_DIR /opt/ros/noetic/share/ompl) #TODO：这里需要修改为你的ompl路径
find_package(ompl REQUIRED)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/hybrid_astar.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/cl_cbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/neighbor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/planresult.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/timer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/environment.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/agvs_plannerv2/agvs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sh_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/agvs.cpp
)
add_custom_target(clang-format
  COMMAND clang-format -style=Google -i ${ALL_SOURCE_FILES} 
)
add_custom_target(everything
  DEPENDS clang-format docs 
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("Release mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
else(${CMAKE_BUILD_TYPE} MATCHES "Release")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif (${CMAKE_BUILD_TYPE} MATCHES "Release") 
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(
  FILES
  ## 运动状态
  EulerAngle.msg
  Position.msg
  DynamicPosition.msg

  ## 小车相关
  CarPhysicalStatus.msg
  CarRouteInfo.msg
  
  ## 仿真机相关
  DroneWayPoint.msg
  DroneWayPointInfo.msg
  DronePhysicalStatus.msg

  ## 场景管理和选手管理相关
  UserPhysicalStatus.msg
  UserCmdRequest.msg
  UserCmdResponse.msg
  BindDrone.msg
  UnBindInfo.msg
  BindCargo.msg
  BillStatus.msg
  DroneMsg.msg
  EventMsg.msg
  PanoramicInfo.msg

  ## 地图相关
  Voxel.msg
)
add_service_files(
  FILES 
  QueryVoxel.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  user_pkg
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES race_demo
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime user_pkg map_generator
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(SH_Astar
  src/sh_astar.cpp
)
target_link_libraries(SH_Astar
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


# 编译读取json文件的库
add_library(config_handler SHARED
  src/Config.cpp
  src/uav_route.cpp
)
target_link_libraries(config_handler
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

add_library(agvs src/agvs.cpp)
target_link_libraries(agvs 
                    ${catkin_LIBRARIES}
                    ${Boost_LIBRARIES}
                    ${OMPL_LIBRARIES}
                    yaml-cpp
                    config_handler
                    )

add_executable(agvs_planner_node src/agvs_planner_node.cpp)
target_link_libraries(agvs_planner_node
                      agvs)     


# 编译race_demo_cpp_node
# add_executable(race_demo_cpp src/race_demo.cpp src/uav_route.cpp)
# add_dependencies(race_demo_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(race_demo_cpp ${catkin_LIBRARIES} config_handler)



# add_executable(test6car
#                 src/test.cpp)

# target_link_libraries(test6car
#                         ${catkin_LIBRARIES}
#                         agvs
#                         )
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/race_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/race_demo_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_race_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
