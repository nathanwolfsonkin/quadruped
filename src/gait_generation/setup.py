import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gait_generation'

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
    description='Gait generation package for quadrupeds',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'empirical_gait_loader = gait_generation.empirical_gait_loader:main',
            'analytical_gait_loader = gait_generation.analytical_gait_loader:main',
            'initial_conditions = gait_generation.initial_conditions:main',
        ],
    },
)
