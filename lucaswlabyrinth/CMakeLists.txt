cmake_minimum_required(VERSION 2.8.3)
project(lucaswlabyrinth)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  roscpp
  roslint
)

roslint_cpp()

catkin_package()

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lucaswlabyrinth src/lucaswlabyrinth.cpp)

target_link_libraries(lucaswlabyrinth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
