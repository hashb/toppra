# this.BUILD_WITH_COLLISION_SUPPORT = "OFF";
#   this.BUILD_WITH_CASADI_SUPPORT = "OFF";
#   this.BUILD_WITH_AUTODIFF_SUPPORT = "OFF";
#   this.BUILD_WITH_EXTRA_SUPPORT = "OFF";
#   this.BUILD_WITH_CODEGEN_SUPPORT = "OFF";
#   this.BUILD_WITH_OPENMP_SUPPORT = "OFF";
#   this.BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT = "OFF";
#   this.INSTALL_DOCUMENTATION = "ON";
#   this.GENERATE_PYTHON_STUBS = "ON";
#   this.BUILD_WITH_ACCELERATE_SUPPORT = "OFF";
#   this.BUILD_WITH_SDF_SUPPORT = "OFF";

# install pixi
curl -fsSL https://pixi.sh/install.sh | sh
source ~/.bashrc

pixi run -e linux configure \
            -DBUILD_ADVANCED_TESTING=OFF \
            -DBUILD_WITH_COLLISION_SUPPORT=On \
            -DBUILD_WITH_CASADI_SUPPORT=ON \
            -DBUILD_WITH_AUTODIFF_SUPPORT=ON \
            -DBUILD_WITH_EXTRA_SUPPORT=ON \
            -DBUILD_WITH_CODEGEN_SUPPORT=OFF \
            -DBUILD_WITH_OPENMP_SUPPORT=ON \
            -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=OFF 

pixi run -e linux cmake --build build --target all
pixi run -e linux cmake --install build
