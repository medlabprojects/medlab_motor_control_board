cmake_minimum_required(VERSION 2.8.3)
project(medlab_motor_control_board)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Qt5Core REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EnableMotor.msg
  McbEncoderCurrent.msg
  McbEncoders.msg
  McbGains.msg
  McbStatus.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   CaptureToken.srv
   ReleaseToken.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

set(${PROJECT_NAME}_SRCS
  src/medlab_motor_control_board/mcb_ros.cpp
)

# NOTE: only need to list headers that require moc'ing (i.e. those that inherit QObject)
set(${PROJECT_NAME}_HDRS
  include/medlab_motor_control_board/mcb_ros.h
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include/medlab_motor_control_board
)

if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${${PROJECT_NAME}_SRCS}
    ${${PROJECT_NAME}_MOCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable(MotorTestSine src/${PROJECT_NAME}/MotorTestSine.cpp)
target_link_libraries(MotorTestSine ${catkin_LIBRARIES})
add_dependencies(MotorTestSine ${PROJECT_NAME}_generate_messages_cpp)

add_executable(MCBHelper src/${PROJECT_NAME}/MCBHelper.cpp)
target_link_libraries(MCBHelper ${catkin_LIBRARIES})
add_dependencies(MCBHelper ${PROJECT_NAME}_generate_messages_cpp)


catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs roscpp
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_medlab_motor_control_board.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
