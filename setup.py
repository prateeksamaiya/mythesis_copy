#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['gym_rotors'],
     package_dir={'': 'src'},
     install_requires=['rospkg','netifaces']
)

setup(**setup_args)