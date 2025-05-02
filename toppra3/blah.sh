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
