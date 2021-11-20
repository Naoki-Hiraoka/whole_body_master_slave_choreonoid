cmake_minimum_required(VERSION 3.0.2)
project(whole_body_master_slave_controller)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_{PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  urdf
  primitive_motion_level_tools
  primitive_motion_level_msgs
  cpp_filters
  )
find_package(choreonoid REQUIRED)

# initialize rtmbuild (generate_messages is called)
rtmbuild_init()

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES EEFFrameConverterServiceSkel EEFFrameConverterServiceStub
#  CATKIN_DEPENDS openrtm_aist
  DEPENDS openrtm_aist
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

###########
## Build ##
###########

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
 )

add_executable(master_controller src/master_controller.cpp)
target_link_libraries(master_controller
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
 )

add_executable(slave_controller src/slave_controller.cpp)
target_link_libraries(slave_controller
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
 )

install(TARGETS master_controller slave_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_subdirectory(rtc)
