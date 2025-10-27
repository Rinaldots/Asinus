# Guia de Resolução de Problemas de Controle no Gazebo

## Problemas Identificados e Soluções

### 1. Nome incorreto dos plugins Gazebo
- **Problema**: Plugins com nomes personalizados `asinus_joint_state_caster` e `asinus_diff_drive` não eram encontrados
- **Solução**: Alterados para nomes padrão do Ignition Gazebo:
  - `ignition::gazebo::systems::JointStatePublisher`
  - `ignition::gazebo::systems::DiffDrive`

### 2. Caminho incorreto para arquivo de configuração
- **Problema**: O caminho estava hardcoded para `/home/r1/ws_crawler/...`
- **Solução**: Alterado para usar `$(find ros2_control_asinus)/config/diffbot_controllers.yaml`

### 3. Parâmetros desalinhados do controlador
- **Problema**: `wheel_separation` e `wheel_radius` não correspondiam aos valores do URDF
- **Solução**: Atualizados para:
  - `wheel_separation: 0.567` (era 0.19)
  - `wheel_radius: 0.079` (era 0.034)

### 4. Caminhos de mesh incompatíveis com Gazebo
- **Problema**: Gazebo não consegue resolver `package://` para meshes
- **Solução**: Criados scripts e launch files alternativos que usam `file://` com caminhos absolutos

### 5. Namespace dinâmico no spawn
- **Problema**: Tópico de descrição estava fixo como `/asinus/robot_description`
- **Solução**: Alterado para usar namespace dinâmico

### 6. Timing de inicialização
- **Problema**: Controladores sendo carregados muito cedo
- **Solução**: Adicionados delays nos spawners

## Como Testar - Versão Corrigida

### 1. Compilar o workspace
```bash
cd /home/r2/Documents/ros/src
colcon build --packages-select asinus_gz ros2_control_asinus asinus_description
source install/setup.bash
```

### 2. Executar a simulação (versão com meshes corrigidos)
```bash
ros2 launch asinus_gz asinus_fixed_meshes.launch.py
```

### 3. Ou usar a versão original (pode ter problemas de visualização de meshes)
```bash
ros2 launch asinus_gz asinus_one_robot.launch.py
```

### 4. Executar diagnóstico
```bash
./ros_asinus/asinus_gz/scripts/diagnose_control.sh
```

### 5. Testar controle manual
```bash
python3 ./ros_asinus/asinus_gz/scripts/test_robot_control.py
```

### 6. Controle via teleop (alternativo)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/asinus/diffbot_base_controller/cmd_vel_unstamped
```

## Scripts de Diagnóstico

### Teste de resolução de meshes
```bash
./ros_asinus/asinus_gz/scripts/test_mesh_resolution.sh
```

### Correção de caminhos de mesh
```bash
python3 ./ros_asinus/asinus_gz/scripts/fix_mesh_paths.py
```

## Verificações Adicionais

Se ainda houver problemas:

1. **Verificar se o Gazebo está rodando corretamente**:
   ```bash
   ign gazebo --version
   ```

2. **Verificar se os plugins estão instalados**:
   ```bash
   ros2 pkg list | grep gz_ros2_control
   ```

3. **Verificar logs do controller_manager**:
   ```bash
   ros2 node info /asinus/controller_manager
   ```

4. **Verificar se as interfaces estão disponíveis**:
   ```bash
   ros2 control list_hardware_interfaces --controller-manager /asinus/controller_manager
   ```

## Estrutura de Tópicos Esperada

Após o launch bem-sucedido, você deve ver estes tópicos:
- `/asinus/diffbot_base_controller/cmd_vel_unstamped` - Para comandos de velocidade
- `/asinus/joint_states` - Estado das juntas
- `/asinus/diffbot_base_controller/odom` - Odometria
- `/tf` - Transformações

## Comandos Úteis para Debug

```bash
# Ver todos os tópicos
ros2 topic list

# Ver controladores ativos
ros2 control list_controllers --controller-manager /asinus/controller_manager

# Monitorar joint states
ros2 topic echo /asinus/joint_states

# Enviar comando manual
ros2 topic pub /asinus/diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```
