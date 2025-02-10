from skbuild import setup

setup(
    name="toppra3",
    version="0.1.0",
    description="TOPPRA trajectory optimization library",
    author="Your Name",
    license="Your License",
    packages=["toppra3"],
    cmake_install_dir="toppra3",
    python_requires=">=3.7",
    extras_require={
        "test": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "numpy>=1.20",
        ],
    },
) 