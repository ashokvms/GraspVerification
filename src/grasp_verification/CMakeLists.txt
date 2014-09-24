cmake_minimum_required(VERSION 2.8.3)
project(grasp_verification)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/ros/include
  ${PROJECT_SOURCE_DIR}/common/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(grasp_verification_node
  ros/src/grasp_verification_node.cpp
  common/src/grasp_verification.cpp
)

add_dependencies(grasp_verification_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

 target_link_libraries(grasp_verification_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
 )

install(TARGETS grasp_verification_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)