"""
setup.py
"""

import os
import distutils
import shutil
import subprocess
import tempfile


from distutils.core import setup, Extension
from distutils.sysconfig import get_config_vars

(opt, ) = get_config_vars('OPT')
os.environ['OPT'] = ' '.join(
    flag for flag in opt.split() if flag != '-Wstrict-prototypes'
)

# attribution: https://github.com/pynbody/pynbody/blob/master/setup.py
def check_for_openmp():
    """Check  whether the default compiler supports OpenMP.
    This routine is adapted from yt, thanks to Nathan
    Goldbaum. See https://github.com/pynbody/pynbody/issues/124"""
    # Create a temporary directory
    tmpdir = tempfile.mkdtemp()
    curdir = os.getcwd()
    os.chdir(tmpdir)

    # Get compiler invocation
    compiler = os.environ.get('CC',
                              distutils.sysconfig.get_config_var('CC'))

    # make sure to use just the compiler name without flags
    compiler = compiler.split()[0]

    # Attempt to compile a test script.
    # See http://openmp.org/wp/openmp-compilers/
    filename = r'test.c'
    with open(filename,'w') as f :
        f.write(
        '#include <omp.h>\n'
        '#include <stdio.h>\n'
        'int main() {\n'
        '#pragma omp parallel\n'
        'printf(\"Hello from thread %d, nthreads %d\\n\", omp_get_thread_num(), omp_get_num_threads());\n'
        '}'
        )

    try:
        with open(os.devnull, 'w') as fnull:
            exit_code = subprocess.call([compiler, '-fopenmp', filename],
                                        stdout=fnull, stderr=fnull)
    except OSError :
        exit_code = 1

    # Clean up
    os.chdir(curdir)
    shutil.rmtree(tmpdir)

    if exit_code == 0:
        return True
    else:
        import multiprocessing, platform
        cpus = multiprocessing.cpu_count()
        if cpus > 1:
            print ("""WARNING
OpenMP support is not available in your default C compiler, even though
your machine has more than one core available.
Some routines in pynbody are parallelized using OpenMP and these will
only run on one core with your current configuration.
""")
            if platform.uname()[0]=='Darwin':
                print ("""Since you are running on Mac OS, it's likely that the problem here
is Apple's Clang, which does not support OpenMP at all. The easiest
way to get around this is to download the latest version of gcc from
here: http://hpc.sourceforge.net. After downloading, just point the
CC environment variable to the real gcc and OpenMP support should
get enabled automatically. Something like this -
sudo tar -xzf /path/to/download.tar.gz /
export CC='/usr/local/bin/gcc'
python setup.py clean
python setup.py build
""")
            print ('Continuing your build without OpenMP...\n')

        return False

have_openmp = check_for_openmp()
openmp_args = ['-fopenmp'] if have_openmp else []

sources = [
    'CSF_wrap.cxx',
    '../lib/c2cdist.cpp',
    '../lib/Cloth.cpp',
    '../lib/CSF.cpp',
    '../lib/Particle.cpp',
    '../lib/point_cloud.cpp',
    '../lib/Rasterization.cpp',
    '../lib/XYZReader.cpp'
]

csf_module = Extension(
    '_CSF',
    sources=sources,
    extra_compile_args=openmp_args,
    extra_link_args=openmp_args
)

setup(
    name = 'CSF',
    version = '1.1.1',
    author      = 'Jianbo Qi',
    description = 'CSF: Ground Filtering based on Cloth Simulation',
    ext_modules = [csf_module],
    py_modules = ['CSF'],
)
