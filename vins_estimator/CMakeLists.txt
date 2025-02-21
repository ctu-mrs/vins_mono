cmake_minimum_required(VERSION 3.5)
project(vins_mono_vins_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3")

set(CATKIN_DEPENDENCIES
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  message_generation
  message_runtime
  )

set(LIBRARIES
  VinsMonoVinsEstimator
  )

set(SRC_FILES
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
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

find_package(Ceres REQUIRED)

find_package(Eigen3)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  )

add_message_files(FILES
  Diagnostics.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

add_library(VinsMonoVinsEstimator
  src/estimator_nodelet.cpp
  ${SRC_FILES}
  )

add_dependencies(VinsMonoVinsEstimator ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(VinsMonoVinsEstimator ${catkin_EXPORTED_TARGETS})

target_link_libraries(VinsMonoVinsEstimator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
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
