cmake_minimum_required(VERSION 2.8.3)
project(path_planner_common)

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
        dubins_curves
        dynamic_reconfigure
        project11_transformations
        geographic_visualization_msgs
        )

find_package(GDAL REQUIRED)

add_message_files(
        FILES
        StateMsg.msg
        DubinsPath.msg
        Plan.msg
)

add_service_files(
        FILES
        UpdateReferenceTrajectory.srv
)

#add_action_files(DIRECTORY action FILES path_planner.action)
generate_messages(DEPENDENCIES geographic_msgs)

include_directories(
        ${catkin_INCLUDE_DIRS}
#        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/executive
#        ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/src/planner/*
        ${dubinscurves_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        include
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES path_planner_state dubins_plan path_planner_utilities
        CATKIN_DEPENDS geometry_msgs roscpp project11
        #  DEPENDS system_lib
)

add_library(path_planner_state
        src/state/State.cpp
        )

add_library(dubins_plan
        src/dubinsPlan/DubinsWrapper.cpp
        src/dubinsPlan/DubinsPlan.cpp
        )

target_link_libraries(dubins_plan path_planner_state)

add_library(path_planner_utilities
        src/utilities/TrajectoryDisplayerHelper.cpp
        )

target_link_libraries(path_planner_utilities path_planner_state)

add_dependencies(path_planner_utilities
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        marine_msgs_gencpp
        project11_transformations_gencpp
        )

#catkin_add_gtest(test_planner test/planner/test_planner.cpp)
#target_link_libraries(test_planner planner dubins)

### Install project namespaced headers
#install(DIRECTORY include/${PROJECT_NAME}
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
#        PATTERN ".svn" EXCLUDE)

#install(TARGETS ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

