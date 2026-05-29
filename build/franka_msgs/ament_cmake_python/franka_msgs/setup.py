from setuptools import find_packages
from setuptools import setup

setup(
    name='franka_msgs',
    version='2.1.0',
    packages=find_packages(
        include=('franka_msgs', 'franka_msgs.*')),
)
