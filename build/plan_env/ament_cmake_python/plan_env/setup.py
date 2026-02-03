from setuptools import find_packages
from setuptools import setup

setup(
    name='plan_env',
    version='1.0.0',
    packages=find_packages(
        include=('plan_env', 'plan_env.*')),
)
