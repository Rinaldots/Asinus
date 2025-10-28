cmake_minimum_required(VERSION 3.8)
project(asinus_hardware_interface)

# Padrão C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings para GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependências ROS2 ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- Dependências Boost ---
find_package(Boost REQUIRED COMPONENTS system thread)

# --- Biblioteca do serviço serial ---
ament_auto_add_library(
  serial_port_service
  SHARED
  src/serial_port_service.cpp
)

# link Boost to the serial_port_service target using variables from FindBoost
target_include_directories(serial_port_service PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(serial_port_service PUBLIC ${Boost_LIBRARIES})

# --- Biblioteca principal do plugin ---
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/asinus_hardware_interface.cpp
)

# Link the serial_port_service target into the main plugin library
target_link_libraries(${PROJECT_NAME} PRIVATE serial_port_service)

# Exporta plugin para pluginlib
pluginlib_export_plugin_description_file(hardware_interface asinus_hardware_interface.xml)

# --- Testes opcionais ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finaliza pacote
ament_auto_package()
