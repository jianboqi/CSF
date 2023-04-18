"""
setup.py
"""
import platform
from setuptools import setup, Extension

# TODO handle macOS clang that won't come with openmp
if platform.system() == "Windows":
	openmp_args = ['/openmp']
	openmp_linking_args=[]
else:
	openmp_args = ['-fopenmp']
	openmp_linking_args=['-fopenmp']

sources = [
    'python/CSF/CSF_wrap.cxx',
    'src/c2cdist.cpp',
    'src/Cloth.cpp',
    'src/CSF.cpp',
    'src/Particle.cpp',
    'src/point_cloud.cpp',
    'src/Rasterization.cpp',
    'src/XYZReader.cpp'
]

include_dirs = ["src/"]

csf_module = Extension(
    name='_CSF',
    sources=sources,
    include_dirs=include_dirs,
    extra_compile_args=openmp_args,
    extra_link_args=openmp_linking_args
)

setup(
    name = 'CSF',
    version = '1.1.2',
    author      = 'Jianbo Qi',
    description = 'CSF: Ground Filtering based on Cloth Simulation',
    package_dir = {"" : "python/CSF"},
    ext_modules = [csf_module],
    py_modules = ['CSF']
)
