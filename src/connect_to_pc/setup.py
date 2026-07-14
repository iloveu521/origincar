import os
from glob import glob

from setuptools import setup


package_name = 'connect_to_pc'

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
    install_requires=[
        'setuptools',
        'flask',
        'requests',
    ],
    zip_safe=True,
    maintainer='iloveu521',
    maintainer_email='iloveu521@todo.todo',
    description='OriginCar HTTP bridge to a PC-side vision-language model.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_pc_bridge = connect_to_pc.car_pc_bridge_node:main',
        ],
    },
)

