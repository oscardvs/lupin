from setuptools import find_packages, setup

package_name = 'lupin_greenhouse_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/greenhouse_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'mdp-greenhouse'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='o.a.e.devos@student.tudelft.nl',
    description='ROS 2 wrapper around the mdp-greenhouse course simulator. '
                'Exposes a GetTagReading service that returns sensor '
                'measurements for a given greenhouse tag.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = lupin_greenhouse_bridge.bridge_node:main',
        ],
    },
)
