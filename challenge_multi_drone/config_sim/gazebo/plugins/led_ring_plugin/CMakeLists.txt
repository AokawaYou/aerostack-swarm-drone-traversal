cmake_minimum_required(VERSION 3.5)
project(led_ring_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
# find_package(gazebo_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED) # Removed as used to interface with gazebo classic and no longer used
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# Referencing: https://github.com/gazebosim/ros_gz_project_template/blob/fortress/ros_gz_example_gazebo/CMakeLists.txt 

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})



# LedRingPlugin
set(PLUGIN_NAME LedRingPlugin)
add_library(${PLUGIN_NAME} SHARED
  src/led_ring.cpp
)
set_property(TARGET ${PLUGIN_NAME} PROPERTY CXX_STANDARD 17)
target_include_directories(${PLUGIN_NAME} PUBLIC include)
target_link_libraries(${PLUGIN_NAME}
)
ament_target_dependencies( ${PLUGIN_NAME}
  rclcpp
  # gazebo_ros
  geometry_msgs
  std_msgs
)
ament_export_libraries(${PLUGIN_NAME})

# ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
# ament_export_dependencies(gazebo_ros)

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/led_ring_plugin.sh.in")

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# [fix7] Harmonic link block ----------------------------------------------------
# Link the gz-sim8 / gz-plugin2 targets that actually exist on this machine.
# (gz-sim8 pulls in gz-common, gz-math, gz-transport, gz-msgs and sdformat.)
if(TARGET gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
  target_link_libraries(${PLUGIN_NAME} gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
endif()
if(TARGET gz-plugin${GZ_PLUGIN_VER}::register)
  target_link_libraries(${PLUGIN_NAME} gz-plugin${GZ_PLUGIN_VER}::register)
elseif(TARGET gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER})
  target_link_libraries(${PLUGIN_NAME} gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER})
endif()
# -------------------------------------------------------------------------------
