#!/bin/bash

set -e
dnf install 'dnf-command(config-manager)' -y
dnf config-manager --set-enabled powertools
dnf install almalinux-release-devel -y
dnf groupinstall "Development Tools" -y
dnf install gcc gcc-c++ -y
dnf install tinyxml2-devel git cmake cmake-filesystem make  -y

mkdir -p /opt/build /opt/install
cd /opt/build

export CMAKE_PREFIX_PATH=/opt/install:$CMAKE_PREFIX_PATH

# install console bridge
git clone https://github.com/ros/console_bridge.git
cd console_bridge
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make
make install

# install urdfdom_headers and urdfdom 
git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make
make install

git clone https://github.com/ros/urdfdom.git
cd urdfdom
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make
make install

git clone --recursive https://github.com/hashb/pinocchio.git -b use-system-gcc
cd pinocchio

# install pixi
curl -fsSL https://pixi.sh/install.sh | sh
source ~/.bashrc

pixi run -e all configure \
            -DBUILD_PYTHON_INTERFACE=OFF \
            -DBUILD_UNIT_TESTS=OFF \
            -DBUILD_WITH_COLLISION_SUPPORT=OFF \
            -DBUILD_WITH_CASADI_SUPPORT=OFF \
            -DBUILD_WITH_AUTODIFF_SUPPORT=OFF

pixi run -e all cmake --build build --target all
pixi run -e all cmake --install build
