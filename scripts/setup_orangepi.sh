#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Setup do Orange Pi 3B para o robo Asinus: ROS 2 + workspace + Tailscale.
# Idempotente: pode rodar varias vezes sem quebrar.
#
# Uso (NO Orange Pi):
#   chmod +x setup_orangepi.sh
#   ./setup_orangepi.sh
#
# Opcionais (variaveis de ambiente):
#   ROS_DISTRO=humble        distro ROS 2 (padrao humble = Ubuntu 22.04)
#   WS=$HOME/asinus_ws       pasta do workspace
#   TS_AUTHKEY=tskey-...      autentica o Tailscale sem abrir navegador
# ---------------------------------------------------------------------------
# NB: sem 'set -u' de proposito - os setup.bash do ROS referenciam variaveis
# nao definidas (ex.: AMENT_TRACE_SETUP_FILES) e quebrariam com nounset.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${WS:-$HOME/asinus_ws}"
REPO="${REPO:-https://github.com/Rinaldots/Asinus.git}"
# So compila o nucleo do robo real (sim/slam/nav ficam de fora por ora - deps pesadas)
BUILD_PKGS="asinus_description asinus_demo_bringup asinus_hardware_interface asinus_sensors"
# Chaves rosdep que nao existem no indice publico (pacotes de terceiros/sim)
SKIP_KEYS="kinect_ros2 cspc_lidar p9n_node gazebo_ros2_control gz_ros2_control rtabmap_slam rtabmap_ros"

log(){ echo -e "\n\033[1;32m==> $*\033[0m"; }

# ---------- 0. Checagens ----------
if [ ! -f /etc/os-release ]; then echo "Sem /etc/os-release - SO nao suportado"; exit 1; fi
. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-jammy}"
log "SO: ${PRETTY_NAME:-?} (codename $CODENAME) | ROS 2: $ROS_DISTRO | WS: $WS"
if ! echo "${ID:-}" | grep -qiE 'ubuntu|debian'; then
  echo "AVISO: script assume Ubuntu/Debian. Continuando mesmo assim em 5s..."; sleep 5
fi

# ---------- 1. Repositorio APT do ROS 2 ----------
log "Configurando repositorio APT do ROS 2"
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release git
sudo add-apt-repository -y universe || true
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ---------- 2. ROS 2 base + ferramentas de build ----------
log "Instalando ROS 2 $ROS_DISTRO (ros-base) + ferramentas"
sudo apt-get update
sudo apt-get install -y \
  ros-"$ROS_DISTRO"-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-smbus2

# ---------- 3. Dependencias dos pacotes que vamos usar ----------
log "Instalando deps ROS (controle + sensores)"
sudo apt-get install -y \
  ros-"$ROS_DISTRO"-ros2-control \
  ros-"$ROS_DISTRO"-ros2-controllers \
  ros-"$ROS_DISTRO"-diff-drive-controller \
  ros-"$ROS_DISTRO"-robot-state-publisher \
  ros-"$ROS_DISTRO"-xacro \
  ros-"$ROS_DISTRO"-imu-filter-madgwick \
  ros-"$ROS_DISTRO"-nmea-navsat-driver \
  ros-"$ROS_DISTRO"-robot-localization \
  ros-"$ROS_DISTRO"-joy \
  ros-"$ROS_DISTRO"-teleop-twist-joy \
  libboost-system-dev

# ---------- 4. rosdep ----------
log "Inicializando rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

# ---------- 5. Workspace ----------
log "Preparando workspace em $WS"
mkdir -p "$WS/src"
if [ -d "$WS/src/Asinus/.git" ]; then
  git -C "$WS/src/Asinus" pull --ff-only || true
else
  git clone "$REPO" "$WS/src/Asinus"
fi

# ---------- 6. rosdep install ----------
log "Resolvendo dependencias do workspace"
# shellcheck disable=SC1090
source /opt/ros/"$ROS_DISTRO"/setup.bash
rosdep install --from-paths "$WS/src" --ignore-src -r -y --skip-keys "$SKIP_KEYS" || true

# ---------- 7. Build (nucleo do robo real) ----------
log "Compilando: $BUILD_PKGS"
cd "$WS"
if colcon build --symlink-install --packages-select $BUILD_PKGS; then
  log "Build OK"
else
  log "AVISO: build falhou (veja os erros acima). Continuando o resto do setup..."
fi

# ---------- 8. Sources no ~/.bashrc ----------
log "Configurando ~/.bashrc"
grep -qxF "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
grep -qxF "source $WS/install/setup.bash" ~/.bashrc || \
  echo "source $WS/install/setup.bash" >> ~/.bashrc

# ---------- 9. Tailscale ----------
log "Instalando Tailscale"
if ! command -v tailscale >/dev/null 2>&1; then
  curl -fsSL https://tailscale.com/install.sh | sh
fi
if [ -n "${TS_AUTHKEY:-}" ]; then
  sudo tailscale up --ssh --authkey "$TS_AUTHKEY"
  log "Tailscale conectado. IP: $(tailscale ip -4 2>/dev/null || echo '?')"
else
  log "Para conectar o Tailscale, rode:  sudo tailscale up --ssh"
  log "e abra no navegador a URL que aparecer para autenticar na sua conta."
fi

log "CONCLUIDO. Abra um novo terminal (ou 'source ~/.bashrc') e teste:"
echo "    ros2 pkg list | grep asinus"
echo "    ros2 launch asinus_sensors asinus_sensors.launch.py   # (quando os sensores chegarem)"
