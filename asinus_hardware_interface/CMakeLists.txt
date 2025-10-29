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

# Boost (asio uses boost::system for older boost builds)
find_package(Boost REQUIRED COMPONENTS system)

# --- Biblioteca principal do plugin ---
# Use explicit filename with extension to avoid ambiguous targets
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/asinus_hardware_interface.cpp
  src/async_serial.cpp
)

# Link Boost system library
target_link_libraries(${PROJECT_NAME} Boost::system)

# Exporta plugin para pluginlib
pluginlib_export_plugin_description_file(hardware_interface asinus_hardware_interface.xml)

# --- Testes opcionais ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
endif()

# Finaliza pacote
ament_auto_package()
