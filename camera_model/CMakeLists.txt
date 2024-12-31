cmake_minimum_required(VERSION 2.8.3)
project(vins_mono_camera_model)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

set(CATKIN_DEPENDENCIES
  roscpp
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Boost REQUIRED COMPONENTS
  filesystem program_options system
  )

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

add_executable(vins_mono_calibration
  src/intrinsic_calib.cc
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc)

add_library(${PROJECT_NAME}
  src/chessboard/Chessboard.cc
  src/calib/CameraCalibration.cc
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/CataCamera.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/sparse_graph/Transform.cc
  src/gpl/gpl.cc
  src/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(vins_mono_calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${PROJECT_NAME} vins_mono_calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/camera_model/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
