from setuptools import find_packages, setup

package_name = 'thermal_radiometric_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Explicitly include the package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'PyQt5'],
    zip_safe=True,
    maintainer='dtc',
    maintainer_email='parvmaheshwari2002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_radiometric_visualizer = thermal_radiometric_visualizer.visualizer:main',
            'offline_visualizer = thermal_radiometric_visualizer.offline_visualizer:main'
        ],
        'rqt_gui.plugins': [
            'thermal_radiometric_visualizer = thermal_radiometric_visualizer.visualizer:ThermalRadiometricVisualizer'
        ],
    },
)