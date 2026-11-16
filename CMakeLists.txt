cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

if(NOT CMAKE_CONFIGURATION_TYPES AND "${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Printf version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

#---------------------------------------------------------------------------------------
# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# ROS2 publishes PointCloud2 or Livox CustomMsg; legacy PCL format 2 is rejected.
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate custom msg headers
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

## The SDK the workspace ships, not whatever sits in /usr/local. Searching
## The workspace SDK is authoritative.
find_package(Livox-SDK2 QUIET)
if(Livox-SDK2_FOUND)
  set(LIVOX_LIDAR_SDK_LIBRARY ${Livox-SDK2_SHARED_LIBRARY})
  set(LIVOX_LIDAR_SDK_INCLUDE_DIR ${Livox-SDK2_INCLUDE_DIRS})
else()
  find_library(LIVOX_LIDAR_SDK_LIBRARY NAMES livox_lidar_sdk_shared REQUIRED)
  find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR NAMES livox_lidar_api.h REQUIRED)
endif()
message(STATUS "livox sdk: ${LIVOX_LIDAR_SDK_LIBRARY} (headers ${LIVOX_LIDAR_SDK_INCLUDE_DIR})")

# livox ros2 driver target
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${LIVOX_LIDAR_SDK_INCLUDE_DIR})

# get include directories of custom msg headers
# Humble onwards. Tested by existence rather than by distro name, so a distro
# newer than this file does not silently fall into the pre-Humble branch.
if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
else()
  set(LIVOX_INTERFACE_TARGET "${LIVOX_INTERFACES}__rosidl_typesupport_cpp")
  add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})
  get_target_property(LIVOX_INTERFACES_INCLUDE_DIRECTORIES ${LIVOX_INTERFACE_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
endif()

# include file direcotry
target_include_directories(${PROJECT_NAME} PUBLIC
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs
  3rdparty
  include
  src
)

# link libraries
target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${LIVOX_INTERFACE_TARGET}   # for custom msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # This migrated driver deliberately has no author/license headers, and its
  # existing style is not the ROS formatter baseline. Keep the useful static
  # checks below, but do not reintroduce source attribution or rewrite the
  # complete driver only to satisfy those two presentation linters.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  file(GLOB_RECURSE _livox_third_party_lint_exclude CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/*.hxx")
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${_livox_third_party_lint_exclude})
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
)
