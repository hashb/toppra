#!/bin/bash

set -e
yum config-manager --set-enabled powertools
yum install almalinux-release-devel -y
# dnf groupinstall "Development Tools" -y
yum install gcc gcc-c++ -y
yum install tinyxml2-devel git cmake cmake-filesystem cmake-data make -y

mkdir -p /opt/build /opt/install
cd /opt/build

export CMAKE_PREFIX_PATH=/opt/install:$CMAKE_PREFIX_PATH
export BOOST_ROOT=/opt/install

# add to .bashrc
echo 'export CMAKE_PREFIX_PATH=/opt/install:$CMAKE_PREFIX_PATH' >> ~/.bashrc
echo 'export BOOST_ROOT=/opt/install' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/install/lib:/opt/install/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

# Download Boost 1.68 source code
wget https://archives.boost.io/release/1.68.0/source/boost_1_68_0.tar.gz

# Extract the archive
tar xzf boost_1_68_0.tar.gz

# Change directory to the extracted folder
cd boost_1_68_0

# Bootstrap the build system
./bootstrap.sh --without-libraries=python

# Build and install Boost
./b2 install -j$(nproc) --prefix=/opt/install

# Create directories for config files
mkdir -p /opt/install/lib/cmake/Boost-1.68.0

# Create a simple BoostConfig.cmake that directly links libraries
cat > /opt/install/lib/cmake/Boost-1.68.0/BoostConfig.cmake << 'EOF'
# Prevent multiple inclusion
if(Boost_CONFIG_INCLUDED)
    return()
endif()
set(Boost_CONFIG_INCLUDED TRUE)

# Boost configuration
set(Boost_VERSION "1.68.0")
set(Boost_FOUND TRUE)
set(Boost_INCLUDE_DIR "/opt/install/include")
set(Boost_INCLUDE_DIRS "/opt/install/include")
set(Boost_LIBRARY_DIR "/opt/install/lib")
set(Boost_LIBRARY_DIRS "/opt/install/lib")

# Define all available components
set(_boost_components 
    chrono 
    date_time 
    filesystem 
    serialization 
    system 
    thread 
    wserialization
    unit_test_framework
)

# Create imported targets for each component
foreach(_comp ${_boost_components})
    if(EXISTS "/opt/install/lib/libboost_${_comp}.so")
        add_library(Boost::${_comp} SHARED IMPORTED)
        set_target_properties(Boost::${_comp} PROPERTIES
            IMPORTED_LOCATION "/opt/install/lib/libboost_${_comp}.so"
            INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
            IMPORTED_NO_SONAME TRUE  # Add this for shared libraries
        )
        
        # Add dependencies (important for linking order)
        if(_comp STREQUAL "thread")
            set_property(TARGET Boost::${_comp} APPEND PROPERTY 
                IMPORTED_LINK_INTERFACE_LIBRARIES Boost::system)
        elseif(_comp STREQUAL "filesystem")
            set_property(TARGET Boost::${_comp} APPEND PROPERTY 
                IMPORTED_LINK_INTERFACE_LIBRARIES Boost::system)
        elseif(_comp STREQUAL "chrono")
            set_property(TARGET Boost::${_comp} APPEND PROPERTY 
                IMPORTED_LINK_INTERFACE_LIBRARIES Boost::system)
        endif()
        
        set(Boost_${_comp}_FOUND TRUE)
        message(STATUS "Found Boost::${_comp}")
        
        # Also set the library variable for compatibility
        set(Boost_${comp}_LIBRARY "${_lib_file}")
    else()
        set(Boost_${_comp}_FOUND FALSE)
    endif()
endforeach()

# Create header-only target
add_library(Boost::boost INTERFACE IMPORTED)
set_target_properties(Boost::boost PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
)

# Set compatibility variables
set(Boost_LIBRARIES "")
foreach(_comp ${Boost_FIND_COMPONENTS})
    if(TARGET Boost::${_comp})
        list(APPEND Boost_LIBRARIES Boost::${_comp})
    else()
        if(Boost_FIND_REQUIRED_${_comp})
            set(Boost_FOUND FALSE)
            message(FATAL_ERROR "Required Boost component ${_comp} not found")
        endif()
    endif()
endforeach()
EOF

# Ensure the library directory is known to the system
export LD_LIBRARY_PATH=/opt/install/lib:/opt/install/lib64:$LD_LIBRARY_PATH

ls /opt/install/lib/

cd /opt/build

# install console bridge
git clone https://github.com/ros/console_bridge.git
cd console_bridge
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make install -j $(nproc)

cd /opt/build

# install urdfdom_headers and urdfdom 
git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make install -j $(nproc)

cd /opt/build

git clone https://github.com/ros/urdfdom.git
cd urdfdom
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make install -j $(nproc)

cd /opt/build
# Install Eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build
cd build
cmake ..
make install -j $(nproc)

cd /opt/build

git clone --recursive https://github.com/hashb/pinocchio.git -b use-system-gcc
cd pinocchio

mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/install/ -DCMAKE_BUILD_TYPE=Release \
  -DGENERATE_PYTHON_STUBS=OFF -DBUILD_WITH_COLLISION_SUPPORT=OFF \
  -DBUILD_WITH_ACCELERATE_SUPPORT=OFF -DBUILD_WITH_CASADI_SUPPORT=OFF \
  -DBUILD_WITH_AUTODIFF_SUPPORT=OFF -DBUILD_WITH_EXTRA_SUPPORT=OFF \
  -DBUILD_WITH_OPENMP_SUPPORT=OFF -DBUILD_WITH_CODEGEN_SUPPORT=OFF \
  -DBUILD_WITH_SDF_SUPPORT=OFF -DBUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT=OFF \
  -DBUILD_BENCHMARK=OFF -DBoost_NO_BOOST_CMAKE=ON -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_UNIT_TESTS=OFF
make install -j $(nproc)

