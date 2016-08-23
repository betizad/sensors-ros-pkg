cmake_minimum_required(VERSION 2.8.3)
project(usbl_comms)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    auv_msgs
    snippets
    seatrac
    misc_msgs
    message_filters)

find_package(Boost REQUIRED COMPONENTS system serialization thread)
###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usbl_comms
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(caddy_comms src/caddy/caddy_messages.cpp
  src/caddy/buddy_usbl.cpp
  src/caddy/surface_usbl.cpp
  src/caddy/diver_modem.cpp
  src/caddy/nav_module.cpp
  src/caddy/buddy_handler.cpp
  src/caddy/surface_handler.cpp
  src/caddy/diver_handler.cpp)
#  src/caddy/command_module.cpp
#  src/caddy/status_handler.cpp)
add_dependencies(caddy_comms auv_msgs_gencpp underwater_msgs_gencpp)

## DEPRECATED
# add_library(usbl_comms src/USBLManager.cpp src/DiverMsg.cpp)
# target_link_libraries(caddy_comms ${catkin_LIBRARIES})

#add_library(uros_comms src/uros/uros_messages.cpp
#    src/uros/uros_modem_controller.cpp
#    src/uros/uros_usbl_controller.cpp)
#target_link_libraries(uros_comms ${catkin_LIBRARIES})

#add_executable(uros_modem src/uros/UROSModem.cpp src/DiverMsg.cpp)
#target_link_libraries(uros_modem ${catkin_LIBRARIES})

#add_executable(packer_test src/test/packer_test.cpp)
#target_link_libraries(packer_test ${catkin_LIBRARIES} ${Boost_SERIALIZATION_LIBRARIES})

#target_link_libraries(encoding_v3_test caddy_comms ${catkin_LIBRARIES}  ${Boost_SERIALIZATION_LIBRARIES})

#add_executable(encoding_v3_test src/test/encoding_v3.cpp)
#add_executable(divermsg_test src/test/divermsg_test.cpp src/DiverMsg.cpp)
#add_executable(encoding_test src/test/bitencoding.cpp src/DiverMsg.cpp)
#add_executable(encoding_v2_test src/test/encoding_v2.cpp)
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
# install(TARGETS usbl_comms usbl_comms_node
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
catkin_add_gtest(${PROJECT_NAME}-structures-test
  src/caddy/caddy_messages.cpp
  test/test_caddy_comms_structures.cpp
  test/test_comms_structures.cpp)
if(TARGET ${PROJECT_NAME}-structures-test)
   target_link_libraries(${PROJECT_NAME}-structures-test ${Boost_SERIALIZATION_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
