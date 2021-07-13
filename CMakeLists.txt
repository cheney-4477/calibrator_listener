cmake_minimum_required(VERSION 3.2)
project(calibrator_listener)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++14 support : OK")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")           # comparison between signed and unsigned integer expressions
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")

message(STATUS "PROJECT_SOURCE_DIR : ${PROJECT_SOURCE_DIR}")

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%s")
message("BUILD_TIMESTAMP = ${BUILD_TIMESTAMP}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        tf
        tf2
        tf2_ros
        std_msgs
        message_filters
        sensor_msgs
        geometry_msgs
        dbrobot_msg
        )

#set(THIRDPARTY_INCLUDE_DIR "/home/moi/MOI_All_Projects/DBRobot_Core/thirdparty/include")
set(THIRDPARTY_INCLUDE_DIR "/home/momo/work/lib/include")

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES calibrator_listener
        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

message("PROJECT NAME = ${PROJECT_NAME}")
add_executable(${PROJECT_NAME}  src/calibrator_listener.cpp src/sensor.cpp)

message("include = ${include}")
message("catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")
message("THIRDPARTY_INCLUDE_DIR = ${THIRDPARTY_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${THIRDPARTY_INCLUDE_DIR}
        )

message("catkin_LIBRARIES = ${catkin_LIBRARIES}")
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
