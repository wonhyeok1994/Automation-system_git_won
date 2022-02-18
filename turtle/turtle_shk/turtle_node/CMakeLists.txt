cmake_minimum_required(VERSION 2.8.3)
project(turtle_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs 
    roscpp
    geometry_msgs
    actionlib
    actionlib_msgs
    move_base_msgs
    rosgraph_msgs
    GUI_turtle
    roslib
)


catkin_package(
LIBRARIES
    actionlib tf2 std_msgs roscpp message_runtime message_generation
    geometry_msgs actionlib_msgs move_base_msgs rosgraph_msgs GUI_turtle
    roslib
CATKIN_DEPENDS 
    std_msgs roscpp geometry_msgs message_runtime
    actionlib_msgs move_base_msgs actionlib tf2 rosgraph_msgs GUI_turtle
    roslib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_node src/turtle_node.cpp)
add_dependencies(turtle_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtle_node ${catkin_LIBRARIES})

add_executable(turtle_sub src/turtle_sub.cpp)
add_dependencies(turtle_sub ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtle_sub ${catkin_LIBRARIES})
