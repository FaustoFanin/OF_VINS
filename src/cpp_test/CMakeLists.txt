cmake_minimum_required(VERSION 2.8.3)
project(cpp_test)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  genmsg
  cv_bridge
  sensor_msgs
  image_transport
  geometry_msgs
  nav_msgs
  hector_uav_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cpp_test
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport hector_uav_msgs tf nav_msgs
  DEPENDS EIGEN3 OpenCV#system_lib
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(vel_estimator src/vel_estimator.cpp)
target_link_libraries(vel_estimator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(vel_estimator ${catkin_EXPORTED_TARGETS})

add_executable(nav_filter src/nav_filter.cpp)
target_link_libraries(nav_filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(of_test src/of_test.cpp)
target_link_libraries(of_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(trajectories src/trajectories.cpp)
target_link_libraries(trajectories ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(trajectories ${catkin_EXPORTED_TARGETS})
