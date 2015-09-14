from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# reads package.xml
setup_args = generate_distutils_setup(
    packages=['o3d3xx_h5'],
    package_dir={'': 'src'},
    )

setup(**setup_args)
