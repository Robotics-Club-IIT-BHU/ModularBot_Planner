from setuptools import setup

setup(name="ModuBot_planner_utils",
version = "0.1",
packages = ['clustering', 'experimental', 'planning'],
install_requires = ['numpy', 'pybullet', 'sklearn' ,'gevent'])