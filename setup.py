import platform
from setuptools import setup, Extension

if platform.system() == "Windows":
    openmp_args = ["/openmp"]
    openmp_linking_args = []
    openmp_macro = [("CSF_USE_OPENMP", None)]
elif platform.system() == "Linux":
    openmp_args = ["-fopenmp"]
    openmp_linking_args = ["-fopenmp"]
    openmp_macro = [("CSF_USE_OPENMP", None)]
else:  # macOS, macOS clang won't come with openmp
    openmp_args = []
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

include_dirs = ["src/"]

csf_module = Extension(
    name="_CSF",
    sources=sources,
    include_dirs=include_dirs,
    extra_compile_args=openmp_args,
    extra_link_args=openmp_linking_args,
    define_macros=openmp_macro
)

setup(
    name="cloth_simulation_filter",
    version="1.1.4",
    author="Jianbo Qi",
    url="http://ramm.bnu.edu.cn/projects/CSF/",
    long_description=readme_content,
    long_description_content_type='text/markdown',
    maintainer="Jianbo Qi",
    maintainer_email="jianboqi@126.com",
    license="Apache-2.0",
    keywords="LiDAR DTM DSM Classification",
    description="CSF: Ground Filtering based on Cloth Simulation",
    package_dir={"": "python/CSF"},
    ext_modules=[csf_module],
    py_modules=["CSF"],
)
