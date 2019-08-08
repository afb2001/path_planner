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
        dubins_curves
        robust_dubins
        )

add_message_files(
        FILES
        StateMsg.msg
        Trajectory.msg
)

add_action_files(DIRECTORY action FILES path_planner.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

catkin_package(
          INCLUDE_DIRS include
        #  LIBRARIES asv_helm
        CATKIN_DEPENDS geometry_msgs roscpp project11
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/executive
        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/planner/*
        ${dubinscurves_INCLUDE_DIRS}
        ${robustdubins_INCLUDE_DIRS}
        include
)

add_library(path_planner_common
        src/common/map/Map.cpp
        src/common/dynamic_obstacles/Distribution.cpp
        src/common/dynamic_obstacles/DynamicObstacle.cpp
        src/common/dynamic_obstacles/DynamicObstaclesManager.cpp
        )

add_library(planner
        src/planner/Planner.cpp
        src/planner/search/Vertex.cpp
        src/planner/search/Edge.cpp
        src/planner/utilities/Path.cpp
        src/planner/utilities/Plan.cpp
        src/planner/utilities/StateGenerator.cpp
        src/planner/SamplingBasedPlanner.cpp
        src/planner/AStarPlanner.cpp
        )

target_link_libraries(planner path_planner_common robust_dubins dubins)

add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp
        src/executive/executive.cpp
        src/executive/path.cpp
        src/executive/communication.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)

#target_link_libraries(Executive Path Communication)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        planner
        path_planner_common
        )

catkin_add_gtest(test_planner test/planner/test_planner.cpp)

target_link_libraries(test_planner planner dubins)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

#install(TARGETS ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

