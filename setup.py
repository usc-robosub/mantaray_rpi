from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['wldvl'],
    package_dir={'':'src/dvl-python/serial'}
)


setup(**d)