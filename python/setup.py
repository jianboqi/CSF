"""
setup.py
"""

from distutils.core import setup, Extension

csf_module = Extension('_CSF',
                           sources=['csf_wrap.cxx',
                           '../CSFDLL/c2cdist.cpp',
                           '../CSFDLL/Cloth.cpp',
                           '../CSFDLL/CSF.cpp',
                           '../CSFDLL/Particle.cpp',
                           '../CSFDLL/point_cloud.cpp',
                           '../CSFDLL/Rasterization.cpp',
                           '../CSFDLL/XYZReader.cpp']
                           )

setup (name = 'CSF',
       version = '1.1',
       author      = "Jianbo Qi",
       description = """CSF: Ground Filtering based on Cloth Simulation""",
       ext_modules = [csf_module],
       py_modules = ["CSF"],
       )