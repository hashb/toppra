git clone --recursive https://github.com/stack-of-tasks/pinocchio.git -b v3.6.0
cd pinocchio

# install pixi
curl -fsSL https://pixi.sh/install.sh | sh
source ~/.bashrc

pixi run -e all configure \
            -DBUILD_ADVANCED_TESTING=OFF \
            -DBUILD_WITH_COLLISION_SUPPORT=On \
            -DBUILD_WITH_CASADI_SUPPORT=ON \
            -DBUILD_WITH_AUTODIFF_SUPPORT=ON \
            -DBUILD_WITH_EXTRA_SUPPORT=ON \
            -DBUILD_WITH_CODEGEN_SUPPORT=OFF \
            -DBUILD_WITH_OPENMP_SUPPORT=ON \
            -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=OFF 

pixi run -e all cmake --build build --target all
pixi run -e all cmake --install build
