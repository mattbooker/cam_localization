cmake_minimum_required(VERSION 2.8.3)
project(cam_localization)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package( catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  tf
)

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/")

find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED) # Required for the Eigen library
find_package(Eigen REQUIRED)	     # Eigen offers matrix/vector arithmetic operations
find_package(aruco REQUIRED)

include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  cv_bridge 
  opencv2 
  std_msgs 
  image_transport 
  tf 
)

add_library(CameraGraphCalibrator src/CameraGraphCalibrator.cpp)
target_link_libraries(CameraGraphCalibrator
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${aruco_LIBS}
)

add_library(CameraGraph src/CameraGraph.cpp)
target_link_libraries(CameraGraph
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${aruco_LIBS}
)

add_executable(localizer src/localization.cpp)
target_link_libraries(localizer
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${aruco_LIBS}
  CameraGraph
)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${aruco_LIBS}
  CameraGraphCalibrator
)
