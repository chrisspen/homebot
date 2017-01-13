from __future__ import print_function
 
from setuptools import setup, find_packages, Command
from setuptools.command.test import test as TestCommand

import os
from shutil import copyfile
import sys
import urllib

CURRENT_DIR = os.path.abspath(os.path.dirname(__file__))

VERSION = open('VERSION').read().strip()

def get_reqs(*fns):
    lst = []
    for fn in fns:
        for package in open(os.path.join(CURRENT_DIR, fn)).readlines():
            package = package.strip()
            if not package:
                continue
            lst.append(package.strip())
    return lst

try:
    copyfile('roles/all/pip-requirements.txt', 'pip-requirements.txt')
except Exception as e:
    pass

setup(
    name="homebot",
    version=VERSION,
    packages=find_packages(where='./src/ros/src'),
#     scripts=['bin/chroniker'],
#     package_data={
#         '': ['roles/all/pip-requirements.txt'],
#         'roles': [
#             '',
#         ],
#         'chroniker': [
#             'static/*/*/*.*',
#             'templates/*.*',
#             'templates/*/*.*',
#             'templates/*/*/*.*',
#             'templates/*/*/*/*.*',
#             'fixtures/*',
#             'tests/fixtures/*',
#         ],
#     },
    author="Chris Spencer",
    author_email="chrisspen@gmail.com",
    description="A low-cost home built robot.",
    license="BSD",
    url="https://github.com/chrisspen/homebot",
    #https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        'Development Status :: 6 - Mature',
        'Environment :: Web Environment',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
#         'Programming Language :: Python :: 3.3',
#         'Programming Language :: Python :: 3.4',
#         'Programming Language :: Python :: 3.5',
        'Framework :: Django',
    ],
    zip_safe=False,
    install_requires=get_reqs('pip-requirements.txt'),
    tests_require=get_reqs('pip-requirements-test.txt'),
)
