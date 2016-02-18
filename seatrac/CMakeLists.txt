cmake_minimum_required(VERSION 2.8.3)
project(seatrac)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  snippets
  std_msgs
  underwater_msgs
  sensor_msgs
  auv_msgs
  geometry_msgs
  tf2_ros
  pluginlib
  dynamic_reconfigure
  roslint
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread serialization)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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

## Generate dynamic reconfigurations in the 'cfg' folder
generate_dynamic_reconfigure_options( 
  cfg/USBLControllerCfg.cfg
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs#   underwater_msgs
# )

# Generate linter
roslint_cpp(src/seatrac_sdk/seatrac_sim.cpp)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES seatrac
#  CATKIN_DEPENDS roscpp snippets std_msgs underwater_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(seatrac_sdk
   src/seatrac_sdk/seatrac_serial.cpp
   src/seatrac_sdk/seatrac_factory.cpp
   src/seatrac_node/status_listener.cpp
   src/seatrac_node/usbl_controller.cpp
   src/seatrac_node/bridge_controller.cpp
   src/seatrac_sdk/serialization.cpp
   src/seatrac_sdk/seatrac_sim.cpp
   src/seatrac_node/nav_listener.cpp
   src/seatrac_node/data_listener.cpp
   src/seatrac_node/pinger.cpp
)


## Declare a cpp executable
add_executable(seatrac_factory_tester src/test/factory_tester.cpp)
add_executable(bitfield_encoding_test src/test/bitfield_encodings.cpp)
add_executable(ac_medium_sim src/ac_medium/ac_medium_sim.cpp)
add_executable(seatrac_core src/seatrac_node/seatrac_core.cpp)
    
add_executable(static_beacon src/static_beacon/static_beacon.cpp)

if (DEFINED catkin_EXPORTED_TARGETS)
 add_dependencies(ac_medium_sim ${catkin_EXPORTED_TARGETS})
 add_dependencies(seatrac_core ${catkin_EXPORTED_TARGETS})
 add_dependencies(seatrac_sdk ${catkin_EXPORTED_TARGETS})
 add_dependencies(static_beacon ${catkin_EXPORTED_TARGETS})
endif()

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(seatrac_node seatrac_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(seatrac_sdk
   ${catkin_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_THREAD_LIBRARY}
   ${Boost_SERIALIZATION_LIBRARY}
   Geographic
)

target_link_libraries(seatrac_core
   ${catkin_LIBRARIES}
   seatrac_sdk
)

target_link_libraries(seatrac_factory_tester
   ${catkin_LIBRARIES}
   ${Boost_SERIALIZATION_LIBRARY}
   seatrac_sdk
)

target_link_libraries(bitfield_encoding_test
   ${catkin_LIBRARIES}
   ${Boost_SERIALIZATION_LIBRARY}
   seatrac_sdk
)

target_link_libraries(ac_medium_sim
   ${catkin_LIBRARIES}
)

target_link_libraries(static_beacon ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS seatrac seatrac_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_seatrac.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
