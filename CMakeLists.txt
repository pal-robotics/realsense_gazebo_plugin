cmake_minimum_required(VERSION 2.8.3)
project(realsense_gazebo_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    cv_bridge
    rostest
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
    ${PROJECT_NAME}
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(test_realsense_streams test/realsense_streams_test.cpp)
target_link_libraries(
    test_realsense_streams
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_dependencies(test_realsense_streams ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
