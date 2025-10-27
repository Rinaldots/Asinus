cmake_minimum_required(VERSION 3.16)
project(ros2_control_asinus LANGUAGES CXX)

# Opções de compilação
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependências necessárias
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Adiciona a biblioteca compartilhada do plugin
add_library(
  ros2_control_asinus
  SHARED
  src/diffbot_system.cpp
)

# Define o padrão C++ e inclui os diretórios corretos
target_compile_features(ros2_control_asinus PUBLIC cxx_std_17)
target_include_directories(ros2_control_asinus PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include/ros2_control_asinus>
)

# Liga as dependências ao alvo
ament_target_dependencies(ros2_control_asinus PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Exporta o plugin para o ROS 2 reconhecer
pluginlib_export_plugin_description_file(hardware_interface asinus_control.xml)

# Instala os headers
install(
  DIRECTORY src/include/
  DESTINATION include/ros2_control_asinus
)

# Instala arquivos de configuração e launch (se houver)
install(
  DIRECTORY launch config
  DESTINATION share/ros2_control_asinus
)

# Instala a biblioteca compilada
install(TARGETS ros2_control_asinus
  EXPORT export_ros2_control_asinus
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testes (opcional)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

# Exportações
ament_export_targets(export_ros2_control_asinus HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable(test_plugin src/test_plugin.cpp)
ament_target_dependencies(test_plugin
  pluginlib
  hardware_interface
  rclcpp
)
target_compile_features(test_plugin PUBLIC cxx_std_17)

install(TARGETS test_plugin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()
