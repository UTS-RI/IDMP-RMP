from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['rmp2'],
    package_dir={'': 'rmp2'}
)
setup(**d)