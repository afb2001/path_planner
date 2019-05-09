cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        geographic_msgs
        marine_msgs
        roscpp
        rosbag
        project11
        genmsg
        actionlib_msgs
        actionlib
        )

add_action_files(DIRECTORY action FILES path_planner.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES asv_helm
        CATKIN_DEPENDS geometry_msgs roscpp project11
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/path_planner/src/executive
)

#add_subdirectory(src/executive)

add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp
        src/executive/executive.cpp
        src/executive/controller.cpp
        src/executive/path.cpp
        src/executive/communication.cpp)

#add_library(Path src/executive/path.cpp)
#add_library(Communication src/executive/communication.cpp)
#add_library(Executive src/executive/executive.cpp)
#add_library(Controller src/executive/controller.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)

#target_link_libraries(Executive Path Communication)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
#        Path
#        Communication
#        Executive
#        Controller
        )

#target_link_libraries(Controller Path)

install(TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

