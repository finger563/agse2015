cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

## Start Global Marker

## End Global Marker

## Check C++11 / C++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
message_generation
## Start Package Marker
## End Package Marker
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#
## Declare ROS messages, services and actions 
#

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

# Generate messages in the 'msg' folder

# Generate services in the 'srv' folder
add_service_files(
  FILES
  gripperRotation.srv

  armRotation.srv

  radialPos.srv

  gripperPos.srv

  verticalPos.srv

)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

#
## catkin specific configuration 
#
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motor_control
#  CATKIN_DEPENDS roscpp std_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

#
## Build 
#

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
## Start Include Directories Marker
## End Include Directories Marker
)

## Declare a cpp library
# add_library(motor_control
#   src/${PROJECT_NAME}/motor_control.cpp
# )

## Declare a cpp executable
# add_executable(motor_control_node src/motor_control_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(motor_control_node motor_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(motor_control_node
#   ${catkin_LIBRARIES}
# )

#
## Install 
#

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS motor_control motor_control_node
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

#
## Testing 
#

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_motor_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(radial_actuator_controller
	    src/motor_control/Component.cpp
            src/motor_control/Logger.cpp
            src/motor_control/radial_actuator_controller.cpp
            ## Start radial_actuator_controller CPP Marker

            ## End radial_actuator_controller CPP Marker
           )
target_link_libraries(radial_actuator_controller
                      ## Start radial_actuator_controller Target Link Libraries Marker

                      ## End radial_actuator_controller Target Link Libraries Marker
                      ${catkin_LIBRARIES}
                      )
add_dependencies(radial_actuator_controller
                 motor_control_generate_messages_cpp
                )

add_library(vertical_actuator_controller
	    src/motor_control/Component.cpp
            src/motor_control/Logger.cpp
            src/motor_control/vertical_actuator_controller.cpp
            ## Start vertical_actuator_controller CPP Marker

            ## End vertical_actuator_controller CPP Marker
           )
target_link_libraries(vertical_actuator_controller
                      ## Start vertical_actuator_controller Target Link Libraries Marker

                      ## End vertical_actuator_controller Target Link Libraries Marker
                      ${catkin_LIBRARIES}
                      )
add_dependencies(vertical_actuator_controller
                 motor_control_generate_messages_cpp
                )

add_library(servo_controller
	    src/motor_control/Component.cpp
            src/motor_control/Logger.cpp
            src/motor_control/servo_controller.cpp
            ## Start servo_controller CPP Marker

            ## End servo_controller CPP Marker
           )
target_link_libraries(servo_controller
                      ## Start servo_controller Target Link Libraries Marker

                      ## End servo_controller Target Link Libraries Marker
                      ${catkin_LIBRARIES}
                      )
add_dependencies(servo_controller
                 motor_control_generate_messages_cpp
                )














