cmake_minimum_required(VERSION 3.5)
project(dynamic_moving_objects)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we don't use add_compile_options with pedantic in message packages
  # because the Python C extensions don't comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# Make ament_package() available.
find_package(ament_cmake REQUIRED)

# Referencing: https://github.com/gazebosim/ros_gz_project_template/blob/fortress/ros_gz_example_gazebo/CMakeLists.txt 

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})



# Build the plugin library
set(PLUGIN_NAME DynamicMovingObjects)
add_library(${PLUGIN_NAME} SHARED
  src/dynamic_object.cpp
)
set_property(TARGET ${PLUGIN_NAME} PROPERTY CXX_STANDARD 17)
target_include_directories(${PLUGIN_NAME} PUBLIC include)
target_link_libraries(${PLUGIN_NAME}
)

# Install header files
install(DIRECTORY include/
        DESTINATION include)

# Install the plugin library
install(TARGETS
  ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install package.xml into the share directory.
# This is a real file (not a dummy) and will force creation of
# the share folder along with the corresponding local_setup.bash.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()

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
