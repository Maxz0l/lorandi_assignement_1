#!/bin/bash
# install_deps.sh — Installation des dépendances système pour lorandi_assignament_1
#
# Usage :
#   chmod +x install_deps.sh
#   ./install_deps.sh

set -e  # arrêt immédiat si une commande échoue

ROS_DISTRO=humble

echo "=== [1/3] Paquets ROS2 système ==="
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    python3-colcon-common-extensions \
    python3-rosdep

echo "=== [2/3] Initialisation rosdep (ignoré si déjà fait) ==="
sudo rosdep init 2>/dev/null || true
rosdep update

echo "=== [3/3] Dépendances déclarées dans package.xml ==="
# À exécuter depuis la racine du workspace (ex: ~/ws_assignments)
# rosdep installe automatiquement tout ce qui est déclaré dans package.xml
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rosdep install --from-paths "$SCRIPT_DIR" --ignore-src -r -y

echo ""
echo "Installation terminée."
echo "Pour builder le package :"
echo "  cd ~/ws_assignments && colcon build --packages-select lorandi_assignament_1"
echo "  source install/setup.bash"
