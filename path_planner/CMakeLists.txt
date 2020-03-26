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
        dynamic_reconfigure
        project11_transformations
        geographic_visualization_msgs
        path_planner_common
        )

find_package(GDAL REQUIRED)

#add_message_files(
#        FILES
#        StateMsg.msg
#        Trajectory.msg
#        DubinsPath.msg
#        Plan.msg
#)

add_action_files(DIRECTORY action FILES path_planner.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

generate_dynamic_reconfigure_options(
        cfg/path_planner.cfg
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/executive
        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/planner/*
        ${dubinscurves_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        include
)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES asv_helm
        CATKIN_DEPENDS geometry_msgs roscpp project11
        #  DEPENDS system_lib
)

add_library(path_planner_common # confusing; should change or move to the new package
        src/common/map/Map.cpp
        src/common/dynamic_obstacles/Distribution.cpp
        src/common/dynamic_obstacles/DynamicObstacle.cpp
        src/common/dynamic_obstacles/DynamicObstaclesManager.cpp
        src/common/map/GeoTiffMap.cpp
        src/common/map/GridWorldMap.cpp
        )

target_link_libraries(path_planner_common ${GDAL_LIBRARIES})

add_library(planner
        src/planner/Planner.cpp
        src/planner/search/Vertex.cpp
        src/planner/search/Edge.cpp
        src/planner/utilities/Path.cpp
        src/planner/utilities/StateGenerator.cpp
        src/planner/SamplingBasedPlanner.cpp
        src/planner/AStarPlanner.cpp
        src/planner/utilities/Ribbon.cpp
        src/planner/utilities/RibbonManager.cpp
        )

add_dependencies(planner path_planner_common)

target_link_libraries(planner path_planner_common path_planner_state dubins_plan dubins)

add_library(executive
        src/executive/executive.cpp
        )

target_link_libraries(executive planner)

add_executable(${PROJECT_NAME}_node src/path_planner_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)
add_dependencies(${PROJECT_NAME}_node project11_transformations_gencpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

add_executable(controller_test_node src/controller_test_node.cpp)
add_dependencies(controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_test_node marine_msgs_gencpp path_planner_utilities)
add_dependencies(controller_test_node project11_transformations_gencpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        executive
        path_planner_utilities
        )

target_link_libraries(controller_test_node
        ${catkin_LIBRARIES}
        executive
        path_planner_utilities
        )

catkin_add_gtest(test_planner test/planner/test_planner.cpp)
target_link_libraries(test_planner planner dubins)

catkin_add_gtest(test_system test/system/test_executive.cpp test/system/NodeStub.cpp)
target_link_libraries(test_system executive)

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

