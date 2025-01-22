cmake_minimum_required(VERSION 3.5)
project(vins_mono_feature_tracker)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

set(CATKIN_DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  vins_mono_camera_model
  )

set(LIBRARIES
  VinsMonoFeatureTracker
  )

set(SRC_FILES
  src/parameters.cpp
  src/feature_tracker.cpp
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(OpenCV REQUIRED)

find_package(Eigen3)

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(${PROJECT_NAME}
  src/feature_tracker_node.cpp
  ${SRC_FILES}
  )

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_library(VinsMonoFeatureTracker
  src/feature_tracker_nodelet.cpp
  ${SRC_FILES}
  )

target_link_libraries(VinsMonoFeatureTracker
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${PROJECT_NAME} ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
