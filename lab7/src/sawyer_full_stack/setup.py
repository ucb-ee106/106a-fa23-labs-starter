"""
Setup of EECS106A Lab 7 python codebase
Author: Chris Correa, Han Nguyen, Shrey Aeron, Mingyang Wang
"""
from setuptools import setup

requirements = []

setup(name='sawyer_full_stack',
      version='0.0.0',
      description='Lab 7 Package for EECS106A',
      author='Mingyang Wang, Shrey Aeron',
      author_email='saltyminty@berkeley.edu, aeron@berkeley.edu',
      package_dir = {'': 'src'},
      packages=['paths', 'controllers', 'utils'],
      install_requires=requirements,
      test_suite='test'
     )
