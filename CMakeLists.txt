cmake_minimum_required(VERSION 2.8.3)
project(imu_filter_madgwick)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_geometry_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs tf2_geometry_msgs
  INCLUDE_DIRS
  LIBRARIES imu_filter
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-madgwick_test
    test/stateless_orientation_test.cpp
    test/madgwick_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}-madgwick_test
    imu_filter
    ${catkin_LIBRARIES}
  )
endif()
