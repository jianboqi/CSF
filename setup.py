import platform
from setuptools import setup, Extension
import numpy

if platform.system() == "Windows":
    openmp_args = ["/openmp", "/std:c++11"]
    openmp_linking_args = []
    openmp_macro = [("CSF_USE_OPENMP", None)]
elif platform.system() == "Linux":
    openmp_args = ["-fopenmp", "-std=c++11"]
    openmp_linking_args = ["-fopenmp"]
    openmp_macro = [("CSF_USE_OPENMP", None)]
else:  # macOS, macOS clang won't come with openmp
    openmp_args = ["-std=c++11"]
    openmp_linking_args = []
    openmp_macro = []

with open("README.md", encoding="utf8") as readme:
    readme_content = readme.read()

sources = [
    "python/CSF/CSF_wrap.cxx",
    "src/c2cdist.cpp",
    "src/Cloth.cpp",
    "src/CSF.cpp",
    "src/Particle.cpp",
    "src/point_cloud.cpp",
    "src/Rasterization.cpp",
    "src/XYZReader.cpp",
]

include_dirs = ["src/", numpy.get_include()]

csf_module = Extension(
    name="_CSF_3DFin",
    sources=sources,
    include_dirs=include_dirs,
    extra_compile_args=openmp_args,
    extra_link_args=openmp_linking_args,
    define_macros=openmp_macro
)

setup(
    name="CSF_3DFin",
    version="1.3.0",
    author="Jianbo Qi",
    author_email="jianboqi@126.com",
    url="http://ramm.bnu.edu.cn/projects/CSF/",
    long_description=readme_content,
    long_description_content_type='text/markdown',
    maintainer="Romain Janvier",
    maintainer_email="romain.janvier@hotmail.fr",
    website="https://github.com/3DFin/CSF-3DFIN",
    license="Apache-2.0",
    keywords="LiDAR DTM DSM Classification",
    description="CSF: Ground Filtering based on Cloth Simulation",
    package_dir={"": "python/CSF"},
    ext_modules=[csf_module],
    py_modules=["CSF_3DFin"],
)
