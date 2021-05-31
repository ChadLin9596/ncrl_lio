cmake_minimum_required(VERSION 2.8.3)
project(ncrl_lio)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  rospy
  tf
  image_transport
  visualization_msgs
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

 message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ncrl_px4
#  CATKIN_DEPENDS roscpp
DEPENDS EIGEN3 PCL
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
 src
 include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## SCANREGISTRATION AND LASERODOMETRY
add_executable(scanRegistration src/aloam/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(laserOdometry src/aloam/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ncrl_lio_preprocess src/ncrl_lio_preprocess.cpp)
target_link_libraries(ncrl_lio_preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ncrl_lio_estimator_node_fusion
    src/ncrl_lio_parameters_fusion.cpp
    src/ncrl_lio_estimator_node_fusion.cpp
    src/ncrl_lio_estimator_new.cpp
    src/ncrl_lio_feature_manager_fusion.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/factor/pose_local_parameterization.cpp)
target_link_libraries(ncrl_lio_estimator_node_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ncrl_pose_graph_node
    src/ncrl_lio_parameters_fusion.cpp
    src/ncrl_pose_graph_node.cpp)
target_link_libraries(ncrl_pose_graph_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
