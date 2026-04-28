from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'novatron_xsite3d_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kolu',
    maintainer_email='antti.kolu@novatron.fi',
    description='This package includes nodes for receiving data from Novatron Xsite 3D machine guidance system through third party MQTT interface',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'kinematic_results_publisher_node = novatron_xsite3d_interface.kinematic_results_publisher_node:main',
        ],
    },
)
