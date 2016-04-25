cmake_minimum_required(VERSION 2.8.3)
project(ntnu_cyborg_coordinator)

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

catkin_package(
    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ntnu_cyborg_coordinator_node src/node.cpp)
add_dependencies(ntnu_cyborg_coordinator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ntnu_cyborg_coordinator_node ${catkin_LIBRARIES})
