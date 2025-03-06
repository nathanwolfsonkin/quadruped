import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'energy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'gait_trajectory'), glob(os.path.join('gait_trajectory', '*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Wolf-Sonkin',
    maintainer_email='nathan.wolfsonkin@cooper.edu',
    description='Energy analysis package for quadrupeds',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_energy_node = energy_pkg.python_energy_node:main',
            'log_gazebo_gait = energy_pkg.log_gazebo_gait:main',
        ],
    },
)
