cmake_minimum_required(VERSION 3.0.2)
project(whole_body_master_slave_choreonoid)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  )

# initialize rtmbuild (generate_messages is called)
rtmbuild_init()

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES CFRControllerServiceSkel CFRControllerServiceStub
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

###########
## Build ##
###########

add_subdirectory(rtc)
