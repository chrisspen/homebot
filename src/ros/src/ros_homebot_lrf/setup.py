#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_homebot_lrf'],
    package_dir={'': 'src'},
#     install_requires=[
#         'Pillow>=2.3.0',
#         'numpy>=1.8.2',
#         'pytest>=1.6.1',
#         'PyYAML>=3.11',
#         'scipy>=0.16.1',
#     ],
)

setup(**d)
