cmake_minimum_required(VERSION 2.8.3)
project(tde_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  # Bổ sung dependency cho ESKF/EKF
  message_filters
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tf roscpp rospy cv_bridge pcl_conversions std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime visualization_msgs message_filters eigen_conversions
  DEPENDS EIGEN3 PCL OpenCV GTSAM
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

# ==========================================
# 1. Các node nguyên bản của TDE-LIO (Đã fix lỗi GTSAM & TBB)
# ==========================================
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES} gtsam tbb)

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES} gtsam tbb)

add_dependencies(${PROJECT_NAME}_imuPreintegration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


# ==========================================
# 2. EKF và ESKF nodes (Tích hợp từ imu_filter_ros_irl)
# ==========================================

# # Build EKF node
# add_executable(imu_ekf_node 
#   src/imu_ekf_node.cpp
#   src/butterworth.cpp
# )
# target_link_libraries(imu_ekf_node
#   ${catkin_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
# )
# add_dependencies(imu_ekf_node
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
# )

# Build ESKF node
add_executable(imu_eskf_node
  src/imu_eskf_node.cpp
  src/butterworth.cpp
)
target_link_libraries(imu_eskf_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(imu_eskf_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)