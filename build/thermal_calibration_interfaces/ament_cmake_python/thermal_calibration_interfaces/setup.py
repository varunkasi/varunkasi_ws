from setuptools import find_packages
from setuptools import setup

setup(
    name='thermal_calibration_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('thermal_calibration_interfaces', 'thermal_calibration_interfaces.*')),
)
