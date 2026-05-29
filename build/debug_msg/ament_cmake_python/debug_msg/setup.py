from setuptools import find_packages
from setuptools import setup

setup(
    name='debug_msg',
    version='1.5.0',
    packages=find_packages(
        include=('debug_msg', 'debug_msg.*')),
)
