cmake_minimum_required(VERSION 3.5)
project(vins_mono_feature_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

set(CATKIN_DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_runtime
  vins_mono_camera_model
  )

set(LIBRARIES
  VinsMonoFeatureTracker
  )

set(SRC_FILES
  src/parameters.cpp
  src/feature_tracker.cpp
  )

option(USE_MRS_LIB "Enable or disable usage of mrs_lib" ON)
if (USE_MRS_LIB)
  set(CATKIN_DEPENDENCIES 
    ${CATKIN_DEPENDENCIES} 
    mrs_lib
    )
  add_definitions(-DUSE_MRS_LIB=1)
else ()
  add_definitions(-DUSE_MRS_LIB=0)
endif ()

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(OpenCV REQUIRED)

find_package(Eigen3)

add_message_files(FILES
  Diagnostics.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

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

add_library(VinsMonoFeatureTracker
  src/feature_tracker_nodelet.cpp
  ${SRC_FILES}
  )

add_dependencies(VinsMonoFeatureTracker ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(VinsMonoFeatureTracker ${catkin_EXPORTED_TARGETS})

target_link_libraries(VinsMonoFeatureTracker
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include msg
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
