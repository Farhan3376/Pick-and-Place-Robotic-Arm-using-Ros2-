from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_pick_place',
    version='0.1.0',
    packages=find_packages(
        include=('ur_pick_place', 'ur_pick_place.*')),
)
