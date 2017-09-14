"""
setup.py
"""

import os
import distutils
import shutil
import subprocess
import tempfile
import platform


from distutils.core import setup, Extension

# have_openmp = check_for_openmp()
# openmp_args = ['-fopenmp'] if have_openmp else []
if platform.system() == "Windows":
	openmp_args = ['/openmp']
	openmp_linking_args=[]
else:
	openmp_args = ['-fopenmp']
	openmp_linking_args=['-fopenmp']

sources = [
    'CSF_wrap.cxx',
    '../src/c2cdist.cpp',
    '../src/Cloth.cpp',
    '../src/CSF.cpp',
    '../src/Particle.cpp',
    '../src/point_cloud.cpp',
    '../src/Rasterization.cpp',
    '../src/XYZReader.cpp'
]

csf_module = Extension(
    '_CSF',
    sources=sources,
    extra_compile_args=openmp_args,
    extra_link_args=openmp_linking_args
)

setup(
    name = 'CSF',
    version = '1.1.1',
    author      = 'Jianbo Qi',
    description = 'CSF: Ground Filtering based on Cloth Simulation',
    ext_modules = [csf_module],
    py_modules = ['CSF'],
)
