cmake_minimum_required(VERSION 2.8.3)
project(cyborg_coordinator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
)

add_service_files(
    FILES
    RequestControl.srv
    ReleaseControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

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
#  LIBRARIES cyborg_coordinator
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cyborg_coordinator_node src/cyborg_coordinator_node.cpp)
add_dependencies(cyborg_coordinator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cyborg_coordinator_node ${catkin_LIBRARIES})
