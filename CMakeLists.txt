cmake_minimum_required(VERSION 2.8.3)
project(depthimage_to_laserscan_slice)

set(CMAKE_CXX_STANDARD 14)

add_definitions(-Wall -g)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nodelet
  roscpp
  sensor_msgs
  cv_bridge
)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/depthimage_to_laserscan_slice_node.cpp
  src/depthimage_to_laserscan_slice.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depthimage_to_laserscan_slice.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
