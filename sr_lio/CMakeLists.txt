cmake_minimum_required(VERSION 3.0.2)
project(sr_lio)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

# 设置自定义安装路径
set(PCL_INSTALL_DIR "/path/to/your/pcl/install")
set(GTSAM_INSTALL_DIR "/path/to/your/gtsam/install")
set(EIGEN3_INSTALL_DIR "/path/to/your/eigen/install")

# 添加自定义路径到 CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH ${PCL_INSTALL_DIR} ${GTSAM_INSTALL_DIR} ${EIGEN3_INSTALL_DIR})

# 添加自定义的 include 和 lib 路径
include_directories(
    ${PCL_INSTALL_DIR}/include
    ${GTSAM_INSTALL_DIR}/include
    ${EIGEN3_INSTALL_DIR}/include
)

link_directories(
    ${PCL_INSTALL_DIR}/lib
    ${GTSAM_INSTALL_DIR}/lib
    ${EIGEN3_INSTALL_DIR}/lib
)

# 设置 Eigen3 的查找路径
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INSTALL_DIR}/include/eigen3)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

include(FetchContent)

FetchContent_Declare(
        tessil 
	SOURCE_DIR ${PROJECT_SOURCE_DIR}/thirdLibrary/tessil-src)

if (NOT tessil_POPULATED)
    set(BUILD_TESTING OFF)
    FetchContent_Populate(tessil)

    add_library(robin_map INTERFACE)
    add_library(tsl::robin_map ALIAS robin_map)

    target_include_directories(robin_map INTERFACE
            "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")

    list(APPEND headers "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_set.h")
    target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")

    if (MSVC)
        target_sources(robin_map INTERFACE
                "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/tsl-robin-map.natvis>")
    endif ()
endif ()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  robin_map
  include
)

add_executable(lio_optimization 
		src/lioOptimization.cpp
		src/optimize.cpp
        	src/cloudMap.cpp
		src/cloudProcessing.cpp 
		src/state.cpp
		src/eskfEstimator.cpp
		src/utility.cpp
		src/parameters.cpp)
target_link_libraries(lio_optimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} robin_map)
