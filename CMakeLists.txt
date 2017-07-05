cmake_minimum_required(VERSION 2.8.3)
project(raspberry_robot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

#find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        #aruco
        #aruco_msgs
        cv_bridge
        image_transport
        nav_msgs
        tf
        )

catkin_package(
        # INCLUDE_DIRS include
        LIBRARIES raspberry_robot aruco aruco_msgs
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

find_library(pigpio REQUIRED)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${ARUCO_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)


add_executable(aruco_simple src/test/aruco_simple.cpp)
target_link_libraries(aruco_simple aruco ${OpenCV_LIBS} )

#add_executable(aruco_odometry_publisher src/aruco_odometry_publisher.cpp)
#target_link_libraries(aruco_odometry_publisher aruco ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(aruco_system_viewer src/aruco_system_viewer.cpp)
target_link_libraries(aruco_system_viewer aruco ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_twist_subscriber src/robot_twist_subscriber.cpp)
target_link_libraries(robot_twist_subscriber ${Boost_LIBRARIES}  ${catkin_LIBRARIES}
        pigpio "${CMAKE_THREAD_LIBS_INIT}")

add_executable(robot_servo_controller src/robot_servo_controller.cpp)
target_link_libraries(robot_servo_controller ${Boost_LIBRARIES}  ${catkin_LIBRARIES}
        pigpio "${CMAKE_THREAD_LIBS_INIT}")