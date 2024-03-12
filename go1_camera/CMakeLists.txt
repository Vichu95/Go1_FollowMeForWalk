cmake_minimum_required(VERSION 3.0.2)
project(go1_camera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  message_generation
  image_transport
  camera_info_manager
  sensor_msgs
  nodelet
  dynamic_reconfigure  
  #  camera_info_manager
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs
)

find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_INCLUDE_DIR})
    message(STATUS ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

set(UnitreeCameraSDK unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
link_directories(${PROJECT_SOURCE_DIR}/lib/amd64/)
else()
link_directories(${PROJECT_SOURCE_DIR}/lib/arm64/)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/camera_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
                    ${catkin_LIBRARIES}
                    ${UnitreeCameraSDK}
                    )
