cmake_minimum_required(VERSION 2.8.3)
project(seatrac)

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

find_package(Boost REQUIRED COMPONENTS system thread serialization)

## Generate dynamic reconfigurations in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/USBLControllerCfg.cfg
)

# Generate linter
roslint_cpp(src/seatrac_sdk/seatrac_sim.cpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seatrac_sdk
  CATKIN_DEPENDS roscpp snippets std_msgs underwater_msgs sensor_msgs geometry_msgs auv_msgs tf2_ros pluginlib dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

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
add_dependencies(seatrac_sdk auv_msgs_gencpp underwater_msgs_gencpp)
target_link_libraries(seatrac_sdk
   ${catkin_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_THREAD_LIBRARY}
   ${Boost_SERIALIZATION_LIBRARY}
   Geographic
)

add_executable(seatrac_factory_tester src/test/factory_tester.cpp)
target_link_libraries(seatrac_factory_tester
  ${catkin_LIBRARIES}
  ${Boost_SERIALIZATION_LIBRARY}
  seatrac_sdk)

add_executable(bitfield_encoding_test src/test/bitfield_encodings.cpp)
target_link_libraries(bitfield_encoding_test
   ${catkin_LIBRARIES}
   ${Boost_SERIALIZATION_LIBRARY}
   seatrac_sdk
)

add_executable(ac_medium_sim src/ac_medium/ac_medium_sim.cpp)
add_dependencies(ac_medium_sim auv_msgs_gencpp underwater_msgs_gencpp)
target_link_libraries(ac_medium_sim ${catkin_LIBRARIES})

add_executable(seatrac_core src/seatrac_node/seatrac_core.cpp)
add_dependencies(seatrac_core auv_msgs_gencpp underwater_msgs_gencpp)
target_link_libraries(seatrac_core ${catkin_LIBRARIES} seatrac_sdk)

add_executable(static_beacon src/static_beacon/static_beacon.cpp)
add_dependencies(static_beacon auv_msgs_gencpp underwater_msgs_gencpp)
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
