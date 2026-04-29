from setuptools import find_packages, setup

package_name = 'lupin_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/mission.launch.py']),
    ],
    install_requires=['setuptools', 'mdp-greenhouse>=1.0.3,<2'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='o.a.e.devos@student.tudelft.nl',
    description='Mission orchestrator that drives the MIRTE Master through the '
                'greenhouse tag-scanning routine via Nav2 and the greenhouse bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_orchestrator = lupin_mission.mission_orchestrator:main',
        ],
    },
)
