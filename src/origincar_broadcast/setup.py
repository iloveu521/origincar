import os
from glob import glob

from setuptools import setup


package_name = 'origincar_broadcast'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iloveu521',
    maintainer_email='iloveu521@todo.todo',
    description='Topic-based speech broadcast manager for OriginCar.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'broadcast_manager = origincar_broadcast.broadcast_manager_node:main',
        ],
    },
)
