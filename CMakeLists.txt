cmake_minimum_required(VERSION 2.8.3)
project(imu_filter_madgwick)

find_package(catkin REQUIRED COMPONENTS geometry_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
  INCLUDE_DIRS
  LIBRARIES imu_filter
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


# create imu_filter library
add_library (imu_filter src/imu_filter.cpp  src/stateless_orientation.cpp)
target_link_libraries(imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS imu_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
