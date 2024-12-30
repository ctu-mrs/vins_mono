cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  camera_model
  )

find_package(OpenCV REQUIRED)

find_package(Eigen3)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(feature_tracker
  src/feature_tracker_node.cpp
  src/parameters.cpp
  src/feature_tracker.cpp
  )

add_dependencies(feature_tracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS feature_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

# install(DIRECTORY include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   )

