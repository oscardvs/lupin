from glob import glob

from setuptools import find_packages, setup

package_name = 'lupin_twin'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='o.a.e.devos@student.tudelft.nl',
    description='Digital-twin node aggregating tag observations into a live '
                'world snapshot and an IDW-interpolated sensor field.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twin_node = lupin_twin.node:main',
        ],
    },
)
