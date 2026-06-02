from glob import glob
from setuptools import find_packages, setup

package_name = 'lupin_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', ['lupin_perception/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='o.a.e.devos@student.tudelft.nl',
    description='AprilTag detection and future flower-detection pipelines for MDP Team Lupin.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_annotator = lupin_perception.tag_annotator:main',
            'yolo_detector = lupin_perception.yolo_detector_node:main',
            'sim_flower_detector = lupin_perception.sim_flower_detector_node:main',
            'perception_aggregator = lupin_perception.perception_aggregator:main',
        ],
    },
)
