cmake_minimum_required(VERSION 2.8.3)
project(part)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(part src/part.cpp)

target_link_libraries(part
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES} 
)

install(TARGETS part
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
