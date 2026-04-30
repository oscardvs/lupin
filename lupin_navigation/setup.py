import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lupin_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='TODO@student.tudelft.nl',
    description='AprilTag-based localisation and path planning for MDP Team Lupin.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
