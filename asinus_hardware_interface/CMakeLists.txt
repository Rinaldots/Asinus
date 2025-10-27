cmake_minimum_required(VERSION 3.8)
project(asinus_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # thread regex

ament_auto_find_build_dependencies()

ament_auto_add_library(
  serial_port_service SHARED
  src/serial_port_service.cpp
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/asinus_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface asinus_hardware_interface.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
