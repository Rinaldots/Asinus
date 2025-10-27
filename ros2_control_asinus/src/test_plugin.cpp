#include <pluginlib/class_loader.hpp>
#include <hardware_interface/system_interface.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader("hardware_interface", "hardware_interface::SystemInterface");

  std::vector<std::string> classes = loader.getDeclaredClasses();

  std::cout << "Plugins encontrados:" << std::endl;
  for (const auto& cls : classes)
  {
    std::cout << " - " << cls << std::endl;
  }

  // Tente criar uma instância do seu plugin
  try
  {
    auto instance = loader.createSharedInstance("asinus_control/DiffBotSystemHardware");
    std::cout << "Plugin 'asinus_control/DiffBotSystemHardware' carregado com sucesso!" << std::endl;
  }
  catch (pluginlib::LibraryLoadException & e)
  {
    std::cerr << "Erro ao carregar plugin: " << e.what() << std::endl;
  }

  return 0;
}
