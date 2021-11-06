add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  primitive_motion_level_msgs
  primitive_motion_level_tools
  cfr_calculator
  cpp_filters
  prioritized_qp
  prioritized_qp_base
  ik_constraint
  )
find_package(choreonoid REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(SupportEEFFixer SHARED
  SupportEEFFixer.cpp
  SupportEEFFixerService_impl.cpp
  InternalWrenchController.cpp
  TiltController.cpp
  )
target_link_libraries(SupportEEFFixer
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  OsqpEigen::OsqpEigen
  )
set_target_properties(SupportEEFFixer PROPERTIES PREFIX "")
add_dependencies(SupportEEFFixer RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(SupportEEFFixerComp SupportEEFFixerComp.cpp)
target_link_libraries(SupportEEFFixerComp SupportEEFFixer)
