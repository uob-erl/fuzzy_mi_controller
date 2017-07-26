cmake_minimum_required(VERSION 2.8.3)
project(status_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  move_base
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


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
#  INCLUDE_DIRS include
#  LIBRARIES status_publisher
  CATKIN_DEPENDS cv_bridge image_transport move_base roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(status_publisher
#   src/${PROJECT_NAME}/status_publisher.cpp
# )

## Declare a cpp executable
 add_executable(status_publisher src/status_publisher.cpp) 
 add_executable(experiment_logger src/experiment_logger.cpp)
 add_executable(cam_compress src/cam_compress.cpp) 
 add_executable(status_sound_publisher_node src/status_sound_publisher_node.cpp) 
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(status_publisher_node status_publisher_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(status_publisher
   ${catkin_LIBRARIES}
 )

target_link_libraries(experiment_logger
   ${catkin_LIBRARIES}
 )

target_link_libraries(cam_compress
   ${catkin_LIBRARIES}
 )

target_link_libraries(status_sound_publisher_node
   ${catkin_LIBRARIES}
 )
