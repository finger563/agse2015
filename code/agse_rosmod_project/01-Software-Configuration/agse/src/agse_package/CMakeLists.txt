cmake_minimum_required(VERSION 2.8.3)
project(agse_package)

## Start Global Marker
# check c++11 / c++0x
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
add_message_files(
  FILES
  sampleState.msg

  samplePosition.msg

  sampleOrientation.msg

  controlInputs.msg

  payloadBayState.msg

  payloadBayPosition.msg

  payloadBayOrientation.msg

  armState.msg

  payloadBayDetectionImages.msg

  sampleDetectionImages.msg

)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  sampleStateFromImage.srv

  verticalPos.srv

  armRotation.srv

  radialPos.srv

  gripperPos.srv

  gripperRotation.srv

  captureImage.srv

  payloadBayStateFromImage.srv

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
#  LIBRARIES agse_package
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
  /usr/include/opencv2
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/opencv2
  ${OpenCV_INCLUDE_DIRS}
## End Include Directories Marker
)

## Declare a cpp library
# add_library(agse_package
#   src/${PROJECT_NAME}/agse_package.cpp
# )

## Declare a cpp executable
# add_executable(agse_package_node src/agse_package_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(agse_package_node agse_package_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(agse_package_node
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
# install(TARGETS agse_package agse_package_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agse_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( arm src/agse_package/arm_main.cpp src/agse_package/Component.cpp src/agse_package/Logger.cpp
                src/agse_package/arm_controller.cpp  
		## Start arm CPP Marker
		src/agse_package/SerialPort.cpp
                ## End arm CPP Marker
              )
target_link_libraries(arm 
                      ## Start arm Target Link Libraries Marker

                      ## End arm Target Link Libraries Marker
                      ${catkin_LIBRARIES})
add_dependencies(arm agse_package_generate_messages_cpp)
add_executable( positioning src/agse_package/positioning_main.cpp src/agse_package/Component.cpp src/agse_package/Logger.cpp
                src/agse_package/radial_actuator_controller.cpp  
                src/agse_package/vertical_actuator_controller.cpp  
		## Start positioning CPP Marker
		src/agse_package/gpio.cpp
		src/agse_package/eqep.cpp
                ## End positioning CPP Marker
              )
target_link_libraries(positioning 
                      ## Start positioning Target Link Libraries Marker

                      ## End positioning Target Link Libraries Marker
                      ${catkin_LIBRARIES})
add_dependencies(positioning agse_package_generate_messages_cpp)
add_executable( user_input src/agse_package/user_input_main.cpp src/agse_package/Component.cpp src/agse_package/Logger.cpp
                src/agse_package/user_input_controller.cpp  
                src/agse_package/user_input_imager.cpp  
		## Start user_input CPP Marker
		src/agse_package/gpio.cpp
                ## End user_input CPP Marker
              )
target_link_libraries(user_input 
                      ## Start user_input Target Link Libraries Marker
                      opencv_core 
                      opencv_imgproc 
                      opencv_highgui 
                      opencv_contrib
                      opencv_legacy
                      opencv_photo
                      opencv_video
                      opencv_videostab
		      opencv_calib3d
		      opencv_ml
                      ## End user_input Target Link Libraries Marker
                      ${catkin_LIBRARIES})
add_dependencies(user_input agse_package_generate_messages_cpp)
add_executable( imager src/agse_package/imager_main.cpp src/agse_package/Component.cpp src/agse_package/Logger.cpp
                src/agse_package/image_sensor.cpp  
		## Start imager CPP Marker
		## End imager CPP Marker
              )
target_link_libraries(imager 
                      ## Start imager Target Link Libraries Marker
		      v4l2
                      ## End imager Target Link Libraries Marker
                      ${catkin_LIBRARIES})
add_dependencies(imager agse_package_generate_messages_cpp)
add_executable( detector src/agse_package/detector_main.cpp src/agse_package/Component.cpp src/agse_package/Logger.cpp
                src/agse_package/image_processor.cpp  
		## Start detector CPP Marker
		src/agse_package/sample_detector.cpp
		src/agse_package/payloadbay_detector.cpp
		# everything below here is required by either sample detector or PB detector
		src/agse_package/Object_Tracker.cpp
		src/aruco/ar_omp.cpp
		src/aruco/arucofidmarkers.cpp
		src/aruco/board.cpp
		src/aruco/boarddetector.cpp
		src/aruco/cameraparameters.cpp
		src/aruco/chromaticmask.cpp
		src/aruco/cvdrawingutils.cpp
		src/aruco/highlyreliablemarkers.cpp
		src/aruco/marker.cpp
		src/aruco/markerdetector.cpp
		src/aruco/subpixelcorner.cpp
                ## End detector CPP Marker
              )
target_link_libraries(detector 
                      ## Start detector Target Link Libraries Marker
                      opencv_core 
                      opencv_imgproc 
                      opencv_highgui 
                      opencv_contrib
                      opencv_legacy
                      opencv_photo
                      opencv_video
                      opencv_videostab
		      opencv_calib3d
		      opencv_ml
                      ## End detector Target Link Libraries Marker
                      ${catkin_LIBRARIES})
add_dependencies(detector agse_package_generate_messages_cpp)
add_executable( servos src/agse_package/servos_main.cpp src/agse_package/Component.cpp src/agse_package/Logger.cpp
                src/agse_package/servo_controller.cpp  
		## Start servos CPP Marker
		src/agse_package/Dynamixel.cpp
		src/agse_package/SerialPort.cpp
		src/agse_package/Utils.cpp
                ## End servos CPP Marker
              )
target_link_libraries(servos 
                      ## Start servos Target Link Libraries Marker

                      ## End servos Target Link Libraries Marker
                      ${catkin_LIBRARIES})
add_dependencies(servos agse_package_generate_messages_cpp)










