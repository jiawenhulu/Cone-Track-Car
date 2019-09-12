cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera)

set(RPLIDAR_SDK_PATH "./sdk/")


FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rosconsole
  sensor_msgs
  std_msgs
  message_generation
)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

FIND_PACKAGE( OpenCV REQUIRED )
AUX_SOURCE_DIRECTORY(src/. DIR_SRCS)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosopencv
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  /usr/include/opencv
  /usr/include/opencv2
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(. DIR_SRCS)
add_subdirectory(
  /home/ubuntu/catkin_ws/src/lidar_camera/src/lib
)

catkin_package()

add_executable(lidar_camera src/lidar_camera.cpp ${RPLIDAR_SDK_SRC} ${DIR_SRCS})
target_link_libraries(lidar_camera opencv_lidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lidar_camera cone ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(lidar_camera a-star ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lidar_camera middlelane ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS lidar_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

