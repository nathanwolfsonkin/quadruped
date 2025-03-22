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
            'low_pass_filter = energy_pkg.filter_torques:main',
            'desired_data_logger = energy_pkg.desired_data_logger:main',
            'raw_data_logger = energy_pkg.raw_data_logger:main',
            'filtered_data_logger = energy_pkg.filtered_data_logger:main',
        ],
    },
)
