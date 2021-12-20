cmake_minimum_required(VERSION 3.0.2)
project(raw_gnss_rtklib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
)

# ADD g2o
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/cmake_modules)
find_package(G2O REQUIRED)
find_package(Ceres REQUIRED)
IF(NOT G2O_FOUND)
   message(FATAL_ERROR "G2O not found.")
ENDIF()
include_directories(
       ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o
       ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/build
)

find_package(Eigen3 3.1.0 REQUIRED)

add_subdirectory(./RTKLIB/)
# add rtklib
add_library(convkml RTKLIB/src/convkml.c)
add_library(convrnx RTKLIB/src/convrnx.c)
add_library(datum RTKLIB/src/datum.c)
add_library(download RTKLIB/src/download.c)
add_library(ephemeris RTKLIB/src/ephemeris.c)
add_library(geoid RTKLIB/src/geoid.c)
add_library(gis RTKLIB/src/gis.c)
add_library(ionex RTKLIB/src/ionex.c)
add_library(lambda RTKLIB/src/lambda.c)
add_library(options RTKLIB/src/options.c)
add_library(pntpos RTKLIB/src/pntpos.cpp)
add_library(postpos RTKLIB/src/postpos.c)
add_library(ppp_ar RTKLIB/src/ppp_ar.c)
add_library(ppp RTKLIB/src/ppp.c)
add_library(preceph RTKLIB/src/preceph.c)
add_library(rcvraw RTKLIB/src/rcvraw.c)
add_library(rinex RTKLIB/src/rinex.c)
add_library(rtcm RTKLIB/src/rtcm.c)
add_library(rtcm2 RTKLIB/src/rtcm2.c)
add_library(rtcm3 RTKLIB/src/rtcm3.c)
add_library(rtcm3e RTKLIB/src/rtcm3e.c)
add_library(rtkcmn RTKLIB/src/rtkcmn.c)
add_library(rtkpos RTKLIB/src/rtkpos.c)
add_library(rtksvr RTKLIB/src/rtksvr.c)
add_library(sbas RTKLIB/src/sbas.c)
add_library(solution RTKLIB/src/solution.c)
add_library(stream RTKLIB/src/stream.c)
add_library(streamsvr RTKLIB/src/streamsvr.c)
add_library(tides RTKLIB/src/tides.c)
add_library(tle RTKLIB/src/tle.c)
## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES raw_gnss_rtklib
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/my_g2o
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/raw_gnss_rtklib.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/raw_gnss_rtklib.cpp src/my_g2o/GPSEdge.cpp src/my_g2o/BiasVertex.cpp src/my_g2o/GPSEdgePrior.cpp src/my_g2o/myOptimization.cpp
./RTKLIB/src/rtkcmn.c
./RTKLIB/src/rtkpos.c
./RTKLIB/src/postpos.c
./RTKLIB/src/ppp.c
./RTKLIB/src/ppp_ar.c
./RTKLIB/src/ephemeris.c
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

SET(G2O_LIBRARIES ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_PCG} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_SOLVER_STRUCTURE_ONLY} ${G2O_SOLVER_EIGEN} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  convkml convrnx datum download ephemeris geoid ionex lambda
  options pntpos postpos ppp_ar ppp preceph  rcvraw rinex
  rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtkpos rtksvr sbas solution
  stream streamsvr tle
  gis tides
  # qzslex
   ${G2O_LIBRARIES}
   ${G2O_TYPES_SLAM2D}
   ${G2O_TYPES_SLAM3D}
   ${CERES_LIBRARIES}
)

target_link_libraries(pntpos
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  # convkml convrnx datum download ephemeris geoid ionex lambda
  # options postpos ppp_ar ppp preceph  rcvraw rinex
  # rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtkpos rtksvr sbas solution
  # stream streamsvr tle
  # gis tides
  # qzslex
  ${G2O_LIBRARIES}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${CERES_LIBRARIES}
)
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_raw_gnss_rtklib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
