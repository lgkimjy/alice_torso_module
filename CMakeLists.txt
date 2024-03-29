################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alice_torso_module)
add_compile_options(-std=c++11)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  alice_msgs
  diagnostic_msgs
  cmake_modules
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  heroehs_math
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alice_torso_module
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    alice_msgs
    diagnostic_msgs
    cmake_modules
    robotis_math
    robotis_framework_common
    robotis_controller_msgs
    heroehs_math
  DEPENDS Boost EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(alice_torso_module
  src/alice_torso_module.cpp
  src/pid_controller.cpp
  src/fifth_order_trajectory_generator.cpp
  src/torso_module_state.cpp
)

add_dependencies(alice_torso_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alice_torso_module  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

